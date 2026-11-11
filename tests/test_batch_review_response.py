import json
import unittest

from learning.batch_review_response import submit
from learning.experiment_pipeline import encoded
from learning import batch_interaction as interaction
from tests import test_batch_interaction as fixtures


class ReviewResponseTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.BatchInteractionTests(); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        f = self.f
        f.clock.time = lambda: f.clock.now + 1000
        f.plan['runtime']['conditionBindings'] = {'close': dict(route='generic', interaction='contact_probe')}
        self.plan = f.private/'plan.json'
        self.review = f.root/'batch-review.json'
        self.request = f.root/'batch-before-review-request.json'
        token = f.verify()
        self.packet = dict(schemaVersion='motionbrain.batch_before_review_request.v1', conditionId='close',
            command=f.condition['command'], sessionId=f.scope['hostClockId'], context=f.context,
            controllerBootId=f.plan['controllerBootId'], outputSequence=100, sourcePath=str(f.root),
            legacySnapshot={}, frames=token.report['frameBindings'])
        self.save()

    def save(self):
        self.plan.write_bytes(encoded(self.f.plan))
        self.request.write_bytes(encoded(self.packet))
        self.review.write_bytes(encoded(self.f.envelope))

    def call(self, **kwargs):
        return submit(self.f.repository, self.plan, self.request, self.review,
                      clock=self.f.clock, operator_reviewed=kwargs.get('operator_reviewed', True))

    def test_before_unknown_is_preserved_and_duplicate_does_not_overwrite(self):
        result = self.call()
        self.assertEqual(result['outcome'], 'unknown')
        self.assertFalse(result['physicalJudgmentGenerated'])
        response = self.f.root/'batch-before-review-response.json'
        raw = response.read_bytes()
        with self.assertRaises(FileExistsError): self.call()
        self.assertEqual(response.read_bytes(), raw)

    def test_operator_acknowledgment_required(self):
        with self.assertRaisesRegex(ValueError, 'operator_judgment'): self.call(operator_reviewed=False)

    def test_frames_not_in_current_request_rejected(self):
        self.packet['frames']['gripper'][0]['pixelSha256'] = '0'*64
        self.save()
        with self.assertRaisesRegex(ValueError, 'frame_not_requested'): self.call()

    def test_stale_review_rejected(self):
        self.f.clock.now += 11
        with self.assertRaises(ValueError): self.call()
        self.assertFalse((self.f.root/'batch-before-review-response.json').exists())

    def test_changed_output_sequence_rejected(self):
        self.packet['outputSequence'] += 1
        self.save()
        with self.assertRaisesRegex(ValueError, 'binding_changed'): self.call()

    def test_after_unknown_preserves_original_source_binding(self):
        self.packet, _, _ = self.f.after_packet()
        self.request = self.f.root/'batch-after-review-request.json'
        raw = encoded(dict(sessionId=self.f.scope['hostClockId']))
        (self.f.root/'session.json').write_bytes(raw)
        self.packet['source'].update(path=str(self.f.root), sha256=interaction.sha(raw))
        self.save()
        self.assertEqual(self.call()['outcome'], 'unknown')
        response = json.loads((self.f.root/'batch-after-review-response.json').read_bytes())
        interaction.validate_after_response(self.packet, response)

    def test_after_changed_source_hash_rejected(self):
        self.packet, _, _ = self.f.after_packet()
        self.request = self.f.root/'batch-after-review-request.json'
        self.packet['source']['path'] = str(self.f.root)
        (self.f.root/'session.json').write_bytes(b'{}')
        self.save()
        with self.assertRaisesRegex(ValueError, 'source_hash_changed'): self.call()


if __name__ == '__main__': unittest.main()
