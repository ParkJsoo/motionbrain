import copy
import unittest

from learning import m1_near_action_model as model
from learning.near_gripper_tip_observer import NearGripperTipObserver
from tests.test_near_gripper_tip_observer import image, encoded


class NearModelTests(unittest.TestCase):
    def report(self, gap, width=55): return NearGripperTipObserver().observe(encoded(image(gap, width)))

    def rows(self):
        rows = []
        for i, (direction, before, after) in enumerate([('close', 140., 120.), ('close', 120., 100.),
                                                      ('open', 100., 120.), ('open', 120., 140.)]):
            b, a = self.report(before), self.report(after)
            rows.append(dict(sessionId=f's{i}', sourceSha256=str(i+1)*64,
                observationContractSha256=b['observationContractSha256'],
                command=dict(joint='gripper', direction=direction, percent=70, durationMs=1000),
                before=b, after=a, sourceEvidence=dict(beforeExternalImageSha256='a'*64,
                    afterExternalImageSha256='b'*64, beforeGripperImageSha256=b['imageSha256'],
                    afterGripperImageSha256=a['imageSha256'])))
        return rows

    def fit(self):
        rows = self.rows(); return model.fit(rows, reference=rows[0]['before'], audit_sources=lambda rows: rows)

    def test_fixed_first_width_target_does_not_follow_later_visible_width(self):
        frozen = self.fit(); current = self.report(80., width=45)
        decision = model.select(frozen, current, copy.deepcopy(model.COMMANDS), commands_used=0)
        self.assertEqual(frozen['targetGapPx'], 51.)
        self.assertEqual(decision['targetGapPx'], 51.)
        self.assertEqual(decision['selected']['kind'], 'action')
        self.assertEqual(decision['selected']['command']['direction'], 'close')
        self.assertFalse(decision['contactVerified']); self.assertFalse(decision['graspVerified'])
        self.assertIn('commandMeanGapErrorPx', decision['selected'])

    def test_budget_six_and_hard_pixel_bounds_stop(self):
        frozen = self.fit()
        exhausted = model.select(frozen, self.report(100.), model.COMMANDS, commands_used=6)
        self.assertEqual(exhausted['selected']['kind'], 'stop')
        self.assertEqual(exhausted['selectionReason'], 'command_budget_exhausted')
        below = model.select(frozen, self.report(40.), model.COMMANDS, commands_used=1)
        self.assertEqual(below['selected']['kind'], 'stop')
        with self.assertRaisesRegex(ValueError, 'predicted_gap_outside_experiment'):
            model.predict(frozen, self.report(60.), model.COMMANDS[0])
        with self.assertRaisesRegex(ValueError, 'predicted_gap_outside_experiment'):
            model.predict(frozen, self.report(140.), model.COMMANDS[1])

    def test_local_before_support_limits_object_XY_and_gap(self):
        frozen = self.fit()
        # Same raw observation with a shifted target object remains a consistent
        # report; its much changed relative state must not reuse close responses.
        current = self.report(140.)
        current['object']['center'][0] += 31.
        current['object']['bbox'][0] += 31
        current['state'][0] += 31.
        with self.assertRaisesRegex(ValueError, 'current_outside_local_support'):
            model.predict(frozen, current, model.COMMANDS[0])

    def test_raw_audit_provenance_reference_and_two_rows_per_command(self):
        rows = self.rows()
        with self.assertRaisesRegex(ValueError, 'source_audit_changed'):
            model.fit(rows, reference=rows[0]['before'], audit_sources=lambda rows: [])
        changed = copy.deepcopy(rows); changed[0]['sourceEvidence']['beforeGripperImageSha256'] = 'f'*64
        with self.assertRaisesRegex(ValueError, 'two_camera_source_bindings'):
            model.fit(changed, reference=changed[0]['before'], audit_sources=lambda rows: rows)
        with self.assertRaisesRegex(ValueError, 'first_before_reference_required'):
            model.fit(rows, reference=rows[1]['before'], audit_sources=lambda rows: rows)
        frozen = model.fit(rows[:1], reference=rows[0]['before'], audit_sources=lambda rows: rows)
        with self.assertRaisesRegex(ValueError, 'two_rows_per_command_required'):
            model.predict(frozen, rows[0]['before'], model.COMMANDS[0])

    def test_visibility_gate_and_reference_mutation_stop(self):
        frozen = self.fit(); current = self.report(100.)
        current['object']['center'][0] = 260.
        current['object']['bbox'][0] = 233
        current['state'][0] = 100.
        current['nearClosingVisibilityGate'].update(horizontalInside=False, passed=False)
        decision = model.select(frozen, current, model.COMMANDS, commands_used=0)
        self.assertEqual(decision['selected']['kind'], 'stop')
        self.assertEqual(decision['selectionReason'], 'current_image_visibility_gate_failed')
        frozen['targetGapPx'] -= 1.
        with self.assertRaisesRegex(ValueError, 'fixed_reference_changed'):
            model.select(frozen, self.report(100.), model.COMMANDS, commands_used=0)


if __name__ == '__main__': unittest.main()
