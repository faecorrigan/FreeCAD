# SPDX-License-Identifier: LGPL-2.1-or-later
# SPDX-FileCopyrightText: 2026 sliptonic <shopinthewoods@gmail.com>
# SPDX-FileNotice: Part of the FreeCAD project.

################################################################################
#                                                                              #
#   FreeCAD is free software: you can redistribute it and/or modify            #
#   it under the terms of the GNU Lesser General Public License as             #
#   published by the Free Software Foundation, either version 2.1              #
#   of the License, or (at your option) any later version.                     #
#                                                                              #
#   FreeCAD is distributed in the hope that it will be useful,                 #
#   but WITHOUT ANY WARRANTY; without even the implied warranty                #
#   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                    #
#   See the GNU Lesser General Public License for more details.                #
#                                                                              #
#   You should have received a copy of the GNU Lesser General Public           #
#   License along with FreeCAD. If not, see https://www.gnu.org/licenses       #
#                                                                              #
################################################################################

"""
Tests for the Haas Next Generation post processor (haas_next_generation_post.py).

What the Haas post shares with the other industrial posts (tape marks, program
number, dwell units, canned cycles, tapping, tool length offset, empty-spindle
park) is tested in TestGenericMultiAxisIndustrial.py. This file pins what the
Haas post adds:

- its defaults and properties, and rigid tapping with S alone (no M29);
- the chip conveyor, the optional stops, safe starts, G187 smoothing and spindle
  speed variation, which are tested on hand-built postables so they do not depend
  on what the job builder puts in each one;
- the VFD coolant pressure and the header's tool cycling and measuring;
- the G268 tilted work plane, dynamic work offsets, rotary clamps, tool centre
  point control and the rotaries' trips home, on a real Job and a rotary machine.
"""

import re
from types import SimpleNamespace
from unittest import mock

import FreeCAD
import Path
import Path.Dressup.Utils as PathDressup
import Path.Op.Custom
import Path.Op.Drilling
import Constants
from CAMTests import PathTestUtils
from CAMTests.TestGenericMultiAxisIndustrial import (
    NEEDS_WORK_PLANES,
    IndustrialPostTestBase,
    _tap,
)
from Path.Post.PostList import Postable
from Path.Post.Processor import PostProcessorFactory, VALID_SCOPES, property_scope
from Path.Post.CAMErrors import CAMValueError
from Machine.models.machine import Machine

try:
    from Path.Post.TiltedWorkPlane import PlaneCommand
    from Machine.models.machine import RotationStrategy
except ImportError:
    PlaneCommand = RotationStrategy = None

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())

POST_NAME = "haas_next_generation"


def _machine_ac(strategy):
    """A table-table AC machine, C carrying A, as TestPathTiltedWorkPlane builds it."""
    from Machine.models.machine import RotaryAxis, AxisRole

    machine = Machine.create_3axis_config()
    machine.name = "Test Haas AC"
    machine.rotary_axes["C"] = RotaryAxis(
        name="C",
        rotation_vector=FreeCAD.Vector(0, 0, 1),
        role=AxisRole.TABLE_ROTARY,
        sequence=0,
    )
    machine.rotary_axes["A"] = RotaryAxis(
        name="A",
        rotation_vector=FreeCAD.Vector(1, 0, 0),
        min_limit=-120,
        max_limit=120,
        role=AxisRole.TABLE_ROTARY,
        parent="C",
        sequence=1,
    )
    machine.kinematics.rotation_strategy = strategy
    machine.output.output_header = False
    machine.output.comments.enabled = False
    machine.output.precision.axis = 3
    machine.output.precision.feed = 3
    machine.output.precision.spindle = 0
    machine.processing.tool_change = False
    machine.postprocessor_properties["uppercase_output"] = False
    return machine


class TestHaasNextGenerationPost(IndustrialPostTestBase):
    """What the Haas post adds, on a three-axis mock job."""

    POST_NAME = POST_NAME
    MACHINE_NAME = "Test Haas"

    def setUp(self):
        super().setUp()
        # The post is shared by every test and remembers whether SSV is on.
        self.post._ssv_active = False

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _module(self):
        # The factory loads a post by file path without registering it in sys.modules,
        # so read the module's names from the globals of the post's own __init__.
        return SimpleNamespace(**type(self.post).__init__.__globals__)

    def _set(self, **properties):
        self.post._machine.postprocessor_properties.update(properties)
        self.post.apply_configuration_bundle()

    def _use_ac_machine(self, strategy=None, **properties):
        machine = _machine_ac(strategy or RotationStrategy.DWO)
        machine.postprocessor_properties.update(properties)
        self.post._machine = machine
        self.post.apply_configuration_bundle()

    @staticmethod
    def _labels(section):
        return [item.label for item in section]

    @staticmethod
    def _text(item):
        return item.data["str"]

    @staticmethod
    def _op_item(label="Op", shape="endmill", proxy=None, stock=0.0, z_stock=0.0):
        """An operation postable carrying just what the Haas expansion reads."""
        tool_controller = SimpleNamespace(Tool=SimpleNamespace(ShapeType=shape))
        source = SimpleNamespace(
            Proxy=proxy if proxy is not None else object(),
            StockToLeave=stock,
            ZStockToLeave=z_stock,
        )
        return Postable(
            item_type="operation",
            label=label,
            path=Path.Path([Path.Command("G0", {"X": 1.0})]),
            source=source,
            data={"tool_controller": tool_controller},
        )

    def _prefix(self, *items):
        """Run only the Haas per-operation additions to the prefix over one section."""
        section = list(items)
        parent = type(self.post).__mro__[1]
        with mock.patch.object(PathDressup, "baseOp", side_effect=lambda op: op), mock.patch.object(
            parent, "_expand_prefix"
        ):
            self.post._expand_prefix([("job", section)])
        return section

    def _trailing(self):
        """Run only the Haas additions to the trailing lines over one section."""
        section = []
        parent = type(self.post).__mro__[1]
        with mock.patch.object(parent, "_expand_trailing_lines"):
            self.post._expand_trailing_lines([("job", section)])
        return section

    # ------------------------------------------------------------------
    # schema
    # ------------------------------------------------------------------

    def test_property_schema_is_well_formed(self):
        """Every property declares a valid scope and reaches the values dict."""
        for prop in self.post.get_full_property_schema():
            self.assertIn(property_scope(prop), VALID_SCOPES, prop["name"])
            if prop["type"] == "choice":
                self.assertIn(prop["default"], prop["choices"], prop["name"])
            self.assertIn(prop["name"].upper(), self.post.values, prop["name"])

    def test_haas_defaults(self):
        schema = {p["name"]: p for p in self.post.get_common_property_schema()}
        self.assertEqual("G90 G94 G17", schema["preamble"]["default"])
        self.assertEqual("M5\nM9\nM30", schema["postamble"]["default"])

    def test_supported_commands_include_the_haas_codes(self):
        schema = {p["name"]: p for p in self.post.get_common_property_schema()}
        supported = schema["supported_commands"]["default"].split("\n")
        for code in ("M0", "M1", "M31", "M33", "G65", "M104", "M105", "M138", "M139", "G17"):
            self.assertIn(code, supported)

    @NEEDS_WORK_PLANES
    def test_plane_command_and_strategies(self):
        """Haas writes the G268 family and can serve both DWO and TWP machines."""
        self.assertEqual(PlaneCommand.G268, self.post.PLANE_COMMAND)
        self.assertEqual(("dwo", "twp"), self.post.ROTATION_STRATEGIES)

    def test_no_rigid_tap_mode_word(self):
        self.assertEqual("", self.post.RIGID_TAP_MODE)

    def test_property_defaults(self):
        props = {p["name"]: p for p in self.post.get_property_schema()}
        defaults = {
            "useTCP": True,
            "useDPMFeeds": False,
            "gotChipConveyor": True,
            "optionalStop": "Off",
            "useSmoothing": "Off",
            "SmoothingRoughingTolerance": 0.5,
            "SmoothingSemiFinishingTolerance": 0.1,
            "SmoothingFinishingTolerance": 0.05,
            "optionallyCycleToolsAtStart": False,
            "measureTools": False,
            "toolArmDrive": False,
            "useSSV": False,
            "safeStartAllOperations": False,
            "useG95forTapping": False,
            "coolantPressure": "None",
            "useClampCodes": True,
            "flat_before_tool_change": False,
            "return_rotaries_home": True,
        }
        for name, default in defaults.items():
            with self.subTest(name):
                self.assertIn(name, props)
                self.assertEqual(default, props[name]["default"])

    def test_property_choices(self):
        props = {p["name"]: p for p in self.post.get_property_schema()}
        self.assertEqual(["Off", "Operation", "Tool Change"], props["optionalStop"]["choices"])
        self.assertEqual(
            ["Off", "Automatic", "Rough", "Medium", "Finish"], props["useSmoothing"]["choices"]
        )
        self.assertEqual(["None", "Low", "Normal", "High"], props["coolantPressure"]["choices"])
        for level in self._module().HAAS_COOLANT_PRESSURES:
            self.assertIn(level, props["coolantPressure"]["choices"])

    def test_use_tcp_sits_after_the_dwell_property(self):
        names = [p["name"] for p in self.post.get_property_schema()]
        self.assertEqual(names.index("dwell_in_milliseconds") + 1, names.index("useTCP"))

    def test_empty_spindle_retract_default_is_commented(self):
        props = {p["name"]: p for p in self.post.get_property_schema()}
        self.assertEqual(
            "(Retract Empty Spindle)\nG28 G91 Z0\nG90", props["end_spindle_empty_retract"]["default"]
        )

    # ------------------------------------------------------------------
    # program frame
    # ------------------------------------------------------------------

    def test_preamble_units_and_postamble(self):
        lines = self.lines([])
        preamble = lines.index("G90 G94 G17")
        self.assertEqual("G21", lines[preamble + 1])
        self.assertEqual(["M5", "M9", "M30", "%"], lines[-4:])

    def test_chip_conveyor_starts_and_stops_the_program(self):
        lines = self.lines(["G0 X1 Y2 Z3"])
        move = lines.index("G0 X1.000 Y2.000 Z3.000")
        self.assertLess(lines.index("M31"), move)
        self.assertLess(move, lines.index("M33"))
        self.assertLess(lines.index("M33"), lines.index("M30"))

    def test_chip_conveyor_comments(self):
        self.post._machine.output.comments.enabled = True
        lines = self.lines([])
        self.assertEqual("(Chip Conveyor On)", lines[lines.index("M31") - 1])
        self.assertEqual("(Stop Chip Conveyor)", lines[lines.index("M33") - 1])

    def test_chip_conveyor_off(self):
        self._set(gotChipConveyor=False)
        lines = self.lines(["G0 X1 Y2 Z3"])
        self.assert_no_line("M31", lines)
        self.assert_no_line("M33", lines)

    def test_conveyor_off_is_added_to_the_trailing_lines(self):
        section = self._trailing()
        self.assertEqual(["Post: conveyor off"], self._labels(section))
        self.assertEqual("(Stop Chip Conveyor)\nM33", self._text(section[0]))

    def test_end_spindle_empty_block_carries_the_haas_comment(self):
        self.assertEqual(
            [
                "M05",
                "(Retract Empty Spindle)",
                "G28 G91 Z0",
                "G90",
                "(End spindle empty)",
                "M6 T0",
            ],
            self.post._end_spindle_empty_block(),
        )

    def test_end_spindle_empty_follows_the_conveyor_off(self):
        self.post._machine.output.comments.enabled = True
        self._set(end_spindle_empty=True)
        lines = self.lines([])
        park = lines.index("M6 T0")
        self.assertEqual("(End spindle empty)", lines[park - 1])
        self.assertLess(lines.index("M33"), park)

    # ------------------------------------------------------------------
    # tapping
    # ------------------------------------------------------------------

    def _assert_s_before_the_cycle(self, lines, cycle_word):
        """S<rpm> precedes the cycle; only a retract-mode word may sit between them."""
        speed = lines.index("S500")
        cycle = next(i for i, l in enumerate(lines) if l.startswith(f"{cycle_word} "))
        self.assertLess(speed, cycle, lines)
        self.assertTrue(all(l in ("G98", "G99") for l in lines[speed + 1 : cycle]), lines)
        return lines[cycle]

    def test_rigid_tap_writes_s_alone_before_the_cycle(self):
        """No M29 on a Haas: S<rpm> before the G84, F is pitch times rpm."""
        lines = [l.strip() for l in self.lines(["G0 X0 Y0 Z10", "G98", _tap()])]
        self.assertFalse(any("M29" in l for l in lines), lines)
        cycle = self._assert_s_before_the_cycle(lines, "G84")
        self.assertEqual("G84 X5.000 Y5.000 Z-10.000 F625.000 R2.000", cycle)
        self.assert_line("G80", lines)

    def test_left_hand_tap_writes_s_alone(self):
        lines = [l.strip() for l in self.lines(["G0 X0 Y0 Z10", _tap("G74")])]
        self.assertFalse(any("M29" in l for l in lines), lines)
        self._assert_s_before_the_cycle(lines, "G74")

    # ------------------------------------------------------------------
    # coolant pressure
    # ------------------------------------------------------------------

    def test_coolant_pressure_off_by_default(self):
        lines = self.lines(["M8", "M9"])
        self.assert_line("M8", lines)
        self.assertFalse(any(l.startswith("M8 P") for l in lines), lines)

    def test_coolant_pressure_levels(self):
        for name, word in (("Low", "P0"), ("Normal", "P1"), ("High", "P2")):
            with self.subTest(name):
                self._set(coolantPressure=name)
                lines = self.lines(["M8", "M9"])
                self.assert_line(f"M8 {word}", lines)

    def test_coolant_pressure_is_not_written_on_coolant_off(self):
        self._set(coolantPressure="High")
        lines = self.lines(["M8", "M9"])
        self.assert_line("M9", lines)

    def test_coolant_pressure_does_not_double_an_explicit_p(self):
        self._set(coolantPressure="High")
        lines = self.lines(["M8 P5"])
        coolant = next(l for l in lines if l.startswith("M8"))
        self.assertEqual(1, coolant.count("P"), coolant)

    # ------------------------------------------------------------------
    # optional stops and safe starts
    # ------------------------------------------------------------------

    def test_optional_stop_off_by_default(self):
        self.post._machine.processing.tool_change = True
        lines = self.lines([])
        self.assert_no_line("M1 (Optional Stop)", lines)

    def test_optional_stop_before_a_tool_change(self):
        self.post._machine.processing.tool_change = True
        self._set(optionalStop="Tool Change")
        lines = self.lines([])
        self.assertEqual("M1 (Optional Stop)", lines[lines.index("M6 T1") - 1])

    def test_optional_stop_between_operations(self):
        self._set(optionalStop="Operation")
        first, second = self._op_item("First"), self._op_item("Second")
        section = self._prefix(first, second)
        self.assertEqual(["First", "Post: op stop", "Second"], self._labels(section))
        self.assertEqual("M1 (Optional Stop)", self._text(section[1]))

    def test_optional_stop_is_not_written_before_the_first_operation(self):
        self._set(optionalStop="Operation")
        section = self._prefix(self._op_item("First"))
        self.assertEqual(["First"], self._labels(section))

    def test_safe_start_repeats_the_preamble_and_units_before_later_operations(self):
        self._set(safeStartAllOperations=True)
        section = self._prefix(self._op_item("First"), self._op_item("Second"))
        self.assertEqual(
            ["First", "Post: safe start", "Post: preamble", "Post: units", "Second"],
            self._labels(section),
        )
        self.assertEqual("(Safe Start Operation)", self._text(section[1]))
        self.assertEqual("G90 G94 G17", self._text(section[2]))

    # ------------------------------------------------------------------
    # spindle speed variation
    # ------------------------------------------------------------------

    def test_ssv_off_by_default(self):
        self._set(gotChipConveyor=False)
        section = self._prefix(self._op_item())
        self.assertEqual(["Op"], self._labels(section))
        self.assertEqual([], self._labels(self._trailing()))

    def test_ssv_on_before_the_first_cutting_operation(self):
        self._set(useSSV=True)
        section = self._prefix(self._op_item())
        self.assertEqual(["Post: SSV on", "Op"], self._labels(section))
        self.assertEqual("M138 (SSV On)", self._text(section[0]))

    def test_ssv_is_switched_on_once(self):
        self._set(useSSV=True)
        section = self._prefix(self._op_item("One"), self._op_item("Two"))
        self.assertEqual(["Post: SSV on", "One", "Two"], self._labels(section))

    def test_ssv_off_before_a_drill(self):
        self._set(useSSV=True)
        section = self._prefix(self._op_item("Mill"), self._op_item("Hole", shape="drill"))
        self.assertEqual(["Post: SSV on", "Mill", "Post: SSV Off", "Hole"], self._labels(section))
        self.assertEqual("M139 (SSV Off)", self._text(section[2]))

    def test_ssv_off_before_a_custom_operation(self):
        self._set(useSSV=True)
        custom = mock.Mock(spec=Path.Op.Custom.ObjectCustom)
        section = self._prefix(self._op_item("Mill"), self._op_item("Custom", proxy=custom))
        self.assertEqual(
            ["Post: SSV on", "Mill", "Post: SSV Off", "Custom"], self._labels(section)
        )

    def test_ssv_is_not_started_for_a_drill(self):
        self._set(useSSV=True)
        section = self._prefix(self._op_item("Hole", shape="drill"))
        self.assertEqual(["Hole"], self._labels(section))

    def test_ssv_off_at_the_end_of_the_program(self):
        self._set(useSSV=True, gotChipConveyor=False)
        section = self._trailing()
        self.assertEqual(["Post: ssv off"], self._labels(section))
        self.assertEqual("M139 (SSV Off)", self._text(section[0]))

    # ------------------------------------------------------------------
    # G187 smoothing
    # ------------------------------------------------------------------

    def test_smoothing_off_by_default(self):
        section = self._prefix(self._op_item())
        self.assertEqual(["Op"], self._labels(section))

    def test_smoothing_levels(self):
        for setting, level in (("Rough", 1), ("Medium", 2), ("Finish", 3)):
            with self.subTest(setting):
                self._set(useSmoothing=setting)
                section = self._prefix(self._op_item())
                self.assertEqual(["Post: smoothing", "Op"], self._labels(section))
                self.assertEqual(f"G187 P{level}", self._text(section[0]))

    def test_automatic_smoothing_follows_the_stock_to_leave(self):
        self._set(useSmoothing="Automatic")
        for stock, level in ((0.6, 1), (0.5, 1), (0.2, 2), (0.1, 2), (0.07, 3), (0.0, 3)):
            with self.subTest(stock=stock):
                section = self._prefix(self._op_item(stock=stock))
                self.assertEqual(f"G187 P{level}", self._text(section[0]))

    def test_automatic_smoothing_uses_the_larger_of_the_two_stocks(self):
        self._set(useSmoothing="Automatic")
        section = self._prefix(self._op_item(stock=0.0, z_stock=0.6))
        self.assertEqual("G187 P1", self._text(section[0]))

    def test_automatic_smoothing_thresholds_are_properties(self):
        self._set(useSmoothing="Automatic", SmoothingRoughingTolerance=1.0)
        section = self._prefix(self._op_item(stock=0.6))
        self.assertEqual("G187 P2", self._text(section[0]))

    def test_drilling_and_custom_operations_get_plain_g187(self):
        self._set(useSmoothing="Finish")
        for proxy in (
            mock.Mock(spec=Path.Op.Drilling.ObjectDrilling),
            mock.Mock(spec=Path.Op.Custom.ObjectCustom),
        ):
            with self.subTest(type(proxy).__name__):
                section = self._prefix(self._op_item(proxy=proxy))
                self.assertEqual("G187", self._text(section[0]))

    # ------------------------------------------------------------------
    # header: tool cycling and measuring
    # ------------------------------------------------------------------

    def test_probing_type(self):
        probing_type = self._module().haas_probing_type
        for tool, with_9023, without in (
            ("endmill", 23, 1),
            ("bullnose", 23, 1),
            ("drill", 12, 2),
            ("tap", 12, 2),
            ("chamfer", 12, 2),
            ("ballend", 12, 2),
            ("probe", 12, 2),
        ):
            with self.subTest(tool):
                self.assertEqual(with_9023, probing_type(tool))
                self.assertEqual(without, probing_type(tool, use9023=False))

    def test_probing_type_refuses_an_unknown_tool(self):
        probing_type = self._module().haas_probing_type
        with self.assertRaisesRegex(ValueError, "unsupported"):
            probing_type("saw", tool="T4 (Saw)")
        with self.assertRaisesRegex(ValueError, "None"):
            probing_type(None)

    def test_header_chip_conveyor_lines(self):
        header = self._module().HaasHeaderBuilder(use_chip_conveyor=True)
        names = [c.Name for c in header.Path.Commands]
        self.assertEqual(["(Chip Conveyor On)", "M31"], names[-2:])

    def test_header_without_a_chip_conveyor(self):
        header = self._module().HaasHeaderBuilder()
        self.assertEqual([], header.Path.Commands)

    def test_header_cycles_the_tools_under_block_delete(self):
        header = self._module().HaasHeaderBuilder(cycle_tools=True)
        header.add_tool(1, "Endmill 6mm", 6.0, 25.0, "endmill")
        commands = header.Path.Commands
        gcode = [c.toGCode() for c in commands]
        self.assertTrue(any("T1=Endmill 6mm" in c.Name for c in commands), gcode)
        self.assertTrue(any(g.startswith("M6") and "T1" in g for g in gcode), gcode)
        self.assertFalse(any(g.startswith("G65") for g in gcode), gcode)
        block_deleted = [c for c in commands if c.Annotations.get(Constants.ANNOT_BLOCK_DELETE)]
        self.assertEqual(3, len(block_deleted), gcode)  # the leading M0, the M6, the trailing M0

    def test_header_measures_the_tools_under_block_delete(self):
        header = self._module().HaasHeaderBuilder(measure_tools=True)
        header.add_tool(1, "Endmill 6mm", 6.0, 25.0, "endmill")
        header.add_tool(2, "Drill 5mm", 5.0, 40.0, "drill")
        probes = [c.toGCode() for c in header.Path.Commands if c.toGCode().startswith("G65")]
        self.assertEqual(2, len(probes), probes)
        for token in ("P9023", "A23", "T1", "H25", "D6"):
            self.assertIn(token, probes[0])
        for token in ("P9023", "A12", "T2", "H40", "D5"):
            self.assertIn(token, probes[1])

    def test_header_skips_probe_tools(self):
        header = self._module().HaasHeaderBuilder(measure_tools=True)
        header.add_tool(3, "Renishaw", None, None, "probe")
        self.assertEqual([], header.Path.Commands)

    def test_header_extends_and_retracts_the_probe_arm(self):
        header = self._module().HaasHeaderBuilder(measure_tools=True, tool_arm_drive=True)
        header.add_tool(1, "Endmill 6mm", 6.0, 25.0, "endmill")
        gcode = [c.toGCode() for c in header.Path.Commands]
        extend = next(i for i, g in enumerate(gcode) if g.startswith("M104"))
        probe = next(i for i, g in enumerate(gcode) if g.startswith("G65"))
        retract = next(i for i, g in enumerate(gcode) if g.startswith("M105"))
        self.assertLess(extend, probe)
        self.assertLess(probe, retract)

    def test_header_without_the_probe_arm(self):
        header = self._module().HaasHeaderBuilder(measure_tools=True)
        header.add_tool(1, "Endmill 6mm", 6.0, 25.0, "endmill")
        gcode = [c.toGCode() for c in header.Path.Commands]
        self.assertFalse(any(g.startswith(("M104", "M105")) for g in gcode), gcode)

    def test_header_refuses_to_measure_an_unknown_tool_type(self):
        header = self._module().HaasHeaderBuilder(measure_tools=True)
        header.add_tool(1, "Saw", 100.0, 3.0, "saw")
        with self.assertRaisesRegex(ValueError, "unsupported"):
            header.Path

    def test_tool_dims_reads_the_tool_controller(self):
        bit = SimpleNamespace(
            ShapeType="Endmill", Diameter=SimpleNamespace(Value=6.0), Length=SimpleNamespace(Value=25.0)
        )
        job = SimpleNamespace(Tools=SimpleNamespace(Group=[SimpleNamespace(ToolNumber=3, Tool=bit)]))
        with mock.patch.object(self.post, "_job", job):
            self.assertEqual((6.0, 25.0, "endmill"), self.post._tool_dims(3))
            self.assertEqual((None, None, None), self.post._tool_dims(9))

    def test_tool_dims_falls_back_to_the_shape_name(self):
        bit = SimpleNamespace(
            ShapeName="Drill", Diameter=SimpleNamespace(Value=5.0), Length=SimpleNamespace(Value=40.0)
        )
        job = SimpleNamespace(Tools=SimpleNamespace(Group=[SimpleNamespace(ToolNumber=1, Tool=bit)]))
        with mock.patch.object(self.post, "_job", job):
            self.assertEqual((5.0, 40.0, "drill"), self.post._tool_dims(1))

    def _tool_postables(self):
        def tool(number):
            return Postable(
                item_type="tool_controller",
                label=f"T{number}: Endmill",
                path=Path.Path([]),
                source=None,
                data={"tool_number": number},
            )

        return [("job", [tool(1), tool(1), tool(2)])]

    def test_build_header_collects_tools_with_their_dimensions(self):
        self._set(optionallyCycleToolsAtStart=True)
        with mock.patch.object(self.post, "_tool_dims", return_value=(6.0, 25.0, "endmill")):
            header = self.post._build_header(self._tool_postables())
        self.assertIsInstance(header, self._module().HaasHeaderBuilder)
        self.assertEqual(
            [(1, "T1: Endmill", 6.0, 25.0, "endmill"), (2, "T2: Endmill", 6.0, 25.0, "endmill")],
            header._tools,
        )

    def test_build_header_passes_the_options_on(self):
        self._set(measureTools=True, toolArmDrive=True, optionallyCycleToolsAtStart=True)
        header = self.post._build_header([("job", [])])
        self.assertTrue(header._measure_tools)
        self.assertTrue(header._cycle_tools)
        self.assertTrue(header._tool_arm_drive)
        self.assertTrue(header._use_chip_conveyor)

    def test_build_header_lists_no_tools_unless_asked(self):
        with mock.patch.object(self.post, "_tool_dims", return_value=(6.0, 25.0, "endmill")):
            header = self.post._build_header(self._tool_postables())
        self.assertEqual([], header._tools)

    # ------------------------------------------------------------------
    # rotary helpers
    # ------------------------------------------------------------------

    def test_moves_rotaries(self):
        moves = self.post._moves_rotaries
        self.assertTrue(moves(Path.Command("G1", {"A": 10.0})))
        self.assertTrue(moves(Path.Command("G1", {"X": 1.0, "C": 90.0})))
        self.assertFalse(moves(Path.Command("G1", {"X": 1.0, "Y": 2.0, "Z": 3.0})))

    @NEEDS_WORK_PLANES
    def test_clamp_codes_for_two_rotary_axes(self):
        self._use_ac_machine()
        unclamp = self.post._clamp_rotaries(engage=False)
        clamp = self.post._clamp_rotaries(engage=True)
        self.assertEqual(["Post: unclamp"], self._labels(unclamp))
        self.assertEqual("M11\nM13", self._text(unclamp[0]))
        self.assertEqual(["Post: clamp"], self._labels(clamp))
        self.assertEqual("M10\nM12", self._text(clamp[0]))

    @NEEDS_WORK_PLANES
    def test_clamp_codes_off_write_nothing(self):
        self._use_ac_machine(useClampCodes=False)
        self.assertEqual([], self.post._clamp_rotaries(engage=False))
        self.assertEqual([], self.post._clamp_rotaries(engage=True))

    def test_clamp_codes_for_a_three_axis_machine(self):
        self.assertEqual([], self.post._clamp_rotaries(engage=False))
        self.assertEqual([], self.post._clamp_rotaries(engage=True))

    @NEEDS_WORK_PLANES
    def test_rotary_home_clears_the_part_then_goes_to_zero_under_clamp_control(self):
        self._use_ac_machine(pre_rotary_move="G53 G0 Z0")
        items = self.post._rotary_home(["A", "C"])
        self.assertEqual(
            ["Post: pre-rotary", "Post: unclamp", "Post: rotaries home", "Post: clamp"],
            self._labels(items),
        )
        self.assertEqual("G0 G53 A0 C0", self._text(items[2]))

    @NEEDS_WORK_PLANES
    def test_pose_change_clamps_the_rotary_move(self):
        """Released after the Pre-Rotary block, engaged before the Post-Rotary block."""
        self._use_ac_machine(pre_rotary_move="G53 G0 Z0", post_rotary_move="(post)")
        items = self.post._pose_change_postables(
            RotationStrategy.DWO, FreeCAD.Placement(), {"A": 10.0, "C": 0.0}, False, True
        )
        self.assertEqual(
            [
                "Post: pre-rotary",
                "Post: unclamp",
                "Rotary positioning",
                "Post: clamp",
                "Post: post-rotary",
            ],
            self._labels(items),
        )

    @NEEDS_WORK_PLANES
    def test_pose_change_without_a_rotary_move_writes_no_clamps(self):
        self._use_ac_machine(pre_rotary_move="G53 G0 Z0")
        items = self.post._pose_change_postables(
            RotationStrategy.DWO, FreeCAD.Placement(), {"A": 0.0, "C": 0.0}, False, False
        )
        self.assertFalse(any("clamp" in label for label in self._labels(items)), items)

    @NEEDS_WORK_PLANES
    def test_pose_change_without_clamp_codes(self):
        self._use_ac_machine(useClampCodes=False, pre_rotary_move="G53 G0 Z0")
        items = self.post._pose_change_postables(
            RotationStrategy.DWO, FreeCAD.Placement(), {"A": 10.0, "C": 0.0}, False, True
        )
        self.assertEqual(["Post: pre-rotary", "Rotary positioning"], self._labels(items))

    @NEEDS_WORK_PLANES
    def test_the_g268_declaration_is_marked(self):
        self._use_ac_machine(RotationStrategy.TWP)
        placement = FreeCAD.Placement(
            FreeCAD.Vector(30, 10, 5), FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), 45)
        )
        declare = self.post._plane_postables("TWP_DECLARE", placement)
        self.assertEqual(1, len(declare))
        self.assertTrue(declare[0].data.get("twp_declare"))
        self.assertTrue(self._text(declare[0]).startswith("G268 "), self._text(declare[0]))
        cancel = self.post._plane_postables("TWP_CANCEL")
        self.assertFalse(any(item.data.get("twp_declare") for item in cancel))

    @NEEDS_WORK_PLANES
    def test_tcp_needs_a_tool_controller_to_name_the_h_word(self):
        self._use_ac_machine()
        item = SimpleNamespace(label="Simultaneous", source=None)
        with self.assertRaisesRegex(CAMValueError, "Tool Center Point"):
            self.post._tcp_postables(item, None)

    @NEEDS_WORK_PLANES
    def test_tcp_brackets_with_g234_and_g49(self):
        self._use_ac_machine()
        item = SimpleNamespace(label="Simultaneous", source=None)
        on, off = self.post._tcp_postables(item, 7)
        self.assertEqual(["Post: unclamp", "Post: TCP on"], self._labels(on))
        self.assertEqual("G234 H7", self._text(on[1]))
        self.assertEqual(
            ["Post: TCP off", "Post: tool length offset", "Post: clamp"], self._labels(off)
        )
        self.assertEqual("G49", self._text(off[0]))


PLANE_PATH = [
    Path.Command("G0", {"X": 0, "Y": 0, "Z": 5}),
    Path.Command("G1", {"X": 10, "Y": 0, "Z": -2, "F": 10}),
    Path.Command("G2", {"X": 20, "Y": 10, "Z": -2, "I": 10, "J": 0}),
]

SIMULTANEOUS_PATH = [
    Path.Command("G0", {"X": 0, "Y": 0, "Z": 5}),
    Path.Command("G1", {"X": 10, "A": 10, "C": 20, "F": 10}),
]

HOME = re.compile(r"^G0 G53 ")


@NEEDS_WORK_PLANES
class TestHaasRotaryStrategies(PathTestUtils.PathTestBase):
    """The Haas post on a Job with a tilted work plane and a rotary machine."""

    def setUp(self):
        import Path.Main.Job as PathJob

        self.maxDiff = None
        self.doc = FreeCAD.newDocument("TestHaasRotaryStrategies")
        self.box = self.doc.addObject("Part::Box", "Box")
        self.box.Length, self.box.Width, self.box.Height = 100, 100, 50
        self.doc.recompute()
        self.job = PathJob.Create("Job", [self.box], None)
        self.doc.recompute()

    def tearDown(self):
        FreeCAD.closeDocument(self.doc.Name)

    def _plane(self, degrees=45, origin=FreeCAD.Vector(30, 10, 5)):
        import Path.Main.Workplane as PathWorkplane

        axis = FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), degrees).multVec(FreeCAD.Vector(0, 0, 1))
        return PathWorkplane.createWorkplaneFromToolAxis(self.job, axis, origin=origin)

    def _op(self, label, plane=None, path=None):
        import Path.Op.Custom as PathCustom

        op = PathCustom.Create(label, parentJob=self.job)
        op.Workplane = plane
        self.doc.recompute()
        op.Path = Path.Path(list(path or PLANE_PATH))
        return op

    def _export(self, machine, **properties):
        machine.postprocessor_properties.update(properties)
        self.job.Proxy.getMachine = lambda: machine
        post = PostProcessorFactory.get_post_processor(self.job, POST_NAME)
        if not hasattr(post, "reinitialize"):
            raise AssertionError(f"The '{POST_NAME}' post did not load: {post}")
        post.reinitialize()
        post._machine = machine
        post.apply_configuration_bundle()
        gcode = "\n".join(g for _, g in post.export2())
        return post, [line.strip() for line in gcode.splitlines()]

    def _two_tools_on_one_face(self, machine, **properties):
        """Two tools on one tilted face: a tool change between the two operations."""
        import Path.Tool.Controller as PathToolController

        machine.processing.tool_change = True
        plane = self._plane()
        first = self._op("First", plane)
        second = self._op("Second", plane)
        tc2 = PathToolController.Create("TC2", tool=first.ToolController.Tool, toolNumber=2)
        self.job.Proxy.addToolController(tc2)
        second.ToolController = tc2
        self.doc.recompute()
        return self._export(machine, **properties)

    # ------------------------------------------------------------------
    # TWP: G268 / G253 / G269
    # ------------------------------------------------------------------

    def test_twp_declares_g268_aligns_with_g253_and_cancels_with_g269(self):
        self._op("Tilted", self._plane())
        _, lines = self._export(_machine_ac(RotationStrategy.TWP), pre_rotary_move="G53 G0 Z0")
        declare = next(i for i, l in enumerate(lines) if l.startswith("G268 "))
        self.assertIn("X30.000 Y10.000 Z5.000", lines[declare])
        # A 45 degree tilt about X: I, J, K are rotations about X, Y, Z, in the order Q123 names.
        self.assertIn("I45.000 J0.000 K0.000 Q123", lines[declare])
        self.assertLess(declare, lines.index("G253"))
        self.assertLess(lines.index("G253"), lines.index("G269"))
        self.assertNotIn("G53.1", lines, "G53.1 is the Fanuc align; a Haas does not know it")
        self.assertIn("G1 X10.000 Y0.000 Z-2.000 F600.000", lines, "the path stays in the plane")

    def test_twp_cancels_before_the_postamble(self):
        self._op("Tilted", self._plane())
        _, lines = self._export(_machine_ac(RotationStrategy.TWP))
        self.assertLess(lines.index("G269"), lines.index("M30"))
        self.assertEqual("%", lines[-1])

    def test_twp_sends_the_rotaries_home_after_the_cancel(self):
        self._op("Tilted", self._plane())
        _, lines = self._export(_machine_ac(RotationStrategy.TWP), pre_rotary_move="G53 G0 Z0")
        home = next(i for i, l in enumerate(lines) if HOME.match(l))
        self.assertLess(lines.index("G269"), home)
        self.assertIn("A0", lines[home])
        self.assertIn("C0", lines[home])
        self.assertLess(lines.index("G53 G0 Z0", lines.index("G269")), home, "clear the part first")

    def test_return_rotaries_home_off(self):
        self._op("Tilted", self._plane())
        _, lines = self._export(_machine_ac(RotationStrategy.TWP), return_rotaries_home=False)
        self.assertFalse(any(HOME.match(l) for l in lines), lines)

    def test_a_flat_program_does_not_send_the_rotaries_home(self):
        self._op("Plain")
        _, lines = self._export(_machine_ac(RotationStrategy.TWP))
        self.assertFalse(any(HOME.match(l) for l in lines), lines)

    def test_flat_before_tool_change_homes_after_the_plane_is_cancelled(self):
        _, lines = self._two_tools_on_one_face(
            _machine_ac(RotationStrategy.TWP),
            flat_before_tool_change=True,
            return_rotaries_home=False,
        )
        homes = [i for i, l in enumerate(lines) if HOME.match(l)]
        self.assertEqual(1, len(homes), lines)
        self.assertLess(lines.index("G269"), homes[0])
        self.assertLess(homes[0], lines.index("M6 T2"))

    def test_flat_before_tool_change_is_off_by_default(self):
        _, lines = self._two_tools_on_one_face(
            _machine_ac(RotationStrategy.TWP), return_rotaries_home=False
        )
        self.assertFalse(any(HOME.match(l) for l in lines), lines)

    # ------------------------------------------------------------------
    # DWO: G254 / G255
    # ------------------------------------------------------------------

    def test_dwo_is_switched_on_after_the_rotary_move_and_off_at_the_end(self):
        self._op("Tilted", self._plane())
        _, lines = self._export(
            _machine_ac(RotationStrategy.DWO),
            pre_rotary_move="G53 G0 Z0",
            return_rotaries_home=False,
        )
        rotary = next(i for i, l in enumerate(lines) if re.match(r"^G0 .*A", l))
        self.assertEqual(1, lines.count("G254"), lines)
        self.assertEqual(1, lines.count("G255"), lines)
        self.assertLess(rotary, lines.index("G254"))
        self.assertTrue(
            any(l.startswith("G1 ") for l in lines[lines.index("G254") : lines.index("G255")])
        )
        self.assertLess(lines.index("G255"), lines.index("M30"))
        self.assertFalse(any(l.startswith("G268") for l in lines), "DWO declares no plane")

    def test_dwo_is_cancelled_before_a_tool_change_and_switched_on_again(self):
        _, lines = self._two_tools_on_one_face(
            _machine_ac(RotationStrategy.DWO), return_rotaries_home=False
        )
        m6 = lines.index("M6 T2")
        on = [i for i, l in enumerate(lines) if l == "G254"]
        off = [i for i, l in enumerate(lines) if l == "G255"]
        self.assertEqual(2, len(on), lines)
        self.assertEqual(2, len(off), lines)
        self.assertLess(on[0], off[0])
        self.assertLess(off[0], m6)
        self.assertLess(m6, on[1])
        self.assertLess(on[1], off[1])

    # ------------------------------------------------------------------
    # rotary clamps
    # ------------------------------------------------------------------

    def test_rotary_brakes_are_released_for_the_move_and_engaged_after(self):
        self._op("Tilted", self._plane())
        _, lines = self._export(
            _machine_ac(RotationStrategy.DWO),
            pre_rotary_move="G53 G0 Z0",
            return_rotaries_home=False,
        )
        rotary = next(i for i, l in enumerate(lines) if re.match(r"^G0 .*A", l))
        self.assertLess(lines.index("G53 G0 Z0"), lines.index("M11"))
        self.assertEqual(lines.index("M11") + 1, lines.index("M13"))
        self.assertLess(lines.index("M13"), rotary)
        self.assertLess(rotary, lines.index("M10"))
        self.assertEqual(lines.index("M10") + 1, lines.index("M12"))

    def test_clamp_codes_off(self):
        self._op("Tilted", self._plane())
        _, lines = self._export(_machine_ac(RotationStrategy.DWO), useClampCodes=False)
        for code in ("M10", "M11", "M12", "M13"):
            self.assertNotIn(code, lines)

    # ------------------------------------------------------------------
    # tool centre point control
    # ------------------------------------------------------------------

    def test_tcp_brackets_a_simultaneous_operation(self):
        self._op("Simultaneous", path=SIMULTANEOUS_PATH)
        _, lines = self._export(_machine_ac(RotationStrategy.DWO), return_rotaries_home=False)
        dwo_off = lines.index("G255")
        tcp_on = lines.index("G234 H1")
        tcp_off = lines.index("G49", tcp_on)
        offset = lines.index("G43 H1", tcp_off)
        self.assertLess(dwo_off, tcp_on, "G234 cannot share the control with G254")
        self.assertLess(lines.index("M11", dwo_off), tcp_on, "a simultaneous path runs unclamped")
        self.assertLess(tcp_on, tcp_off)
        self.assertLess(tcp_off, offset)
        self.assertLess(offset, lines.index("M10", tcp_on))

    def test_tcp_off_leaves_a_simultaneous_operation_alone(self):
        self._op("Simultaneous", path=SIMULTANEOUS_PATH)
        _, lines = self._export(_machine_ac(RotationStrategy.DWO), useTCP=False)
        self.assertFalse(any(l.startswith("G234") for l in lines), lines)

    def test_an_ordinary_operation_gets_no_tcp(self):
        self._op("Tilted", self._plane())
        _, lines = self._export(_machine_ac(RotationStrategy.DWO))
        self.assertFalse(any(l.startswith("G234") for l in lines), lines)