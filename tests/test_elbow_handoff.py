from copy import deepcopy
import unittest

from learning.elbow_handoff import make_elbow_receipt, verify_elbow_handoff, PHASE, PROFILE


class ElbowHandoffTests(unittest.TestCase):
    def setUp(self):
        self.boot, self.trial, self.key = "a"*32, "b"*32, b"k"*32
        self.owner = dict(bootId=self.boot, trialId=self.trial, phase=PHASE, elbowDownOnceProfile=PROFILE,
                          seq=1, controlGeneration=12, reason="completed", active=False, failed=False,
                          elbowFirstWriteOffsetUs=50000, elbowZeroWriteOffsetUs=350005)
        self.status = dict(schemaVersion="collection.v1", state="IDLE",
                           motors={f"M{i}": dict(enabled=False, speed=0) for i in range(1, 6)},
                           outputProof=dict(bootId=self.boot, schemaVersion="driver_cursor.v1",
                                            sequenceBefore=111, sequenceAfter=111, stable=True))
        self.entries = [dict(sequence=101+i, atMs=ms, motor=3, pwm=pwm, reason="driver_write")
                        for i, (ms, pwm) in enumerate(zip([150,200,250,300,350,450],[10,20,30,40,50,0]))]
        self.entries += [dict(sequence=106+i, atMs=475, motor=i, pwm=0, reason="forced_off")
                         for i in range(1, 6)]

    def issue(self):
        return make_elbow_receipt(self.key, boot=self.boot, trial=self.trial, generation=12,
                                  start_cursor=100, owner=self.owner, status=self.status, entries=self.entries)

    def check(self, r, **change):
        args = dict(boot=self.boot, owner=self.owner, status=self.status, entries=[])
        return verify_elbow_handoff(r, self.key, **dict(args, **change))

    def test_normal_and_invalidation(self):
        receipt = self.issue()
        self.assertEqual(self.check(receipt),111)
        for changes in (dict(boot="e"*32), dict(entries=self.entries[-1:]),
                        *(dict(owner=dict(self.owner, **{k: v})) for k, v in [
                            ("controlGeneration",13), ("seq",True), ("failed",True), ("reason","interrupted"),
                            ("active",True), ("elbowFirstWriteOffsetUs",49999), ("elbowZeroWriteOffsetUs",366000),
                            ("trialId","1"*32), ("phase","approach"), ("elbowDownOnceProfile","other")] )):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.check(receipt, **changes)
        forged = deepcopy(receipt)
        forged["fields"]["idleCursor"] = 112
        with self.assertRaises(ValueError):
            self.check(forged)
        self.status["motors"]["M3"]["enabled"] = True
        with self.assertRaises(ValueError):
            self.check(receipt)

    def test_ordered_driver_and_clean_tail(self):
        original = self.entries
        for index, fields in [(5, {"reason":"forced_off"}), (2, {"motor":2}), (6, {"pwm":10}), (6, {"atMs":449}),
                              (2,{"sequence":110}), (5, {"atMs":900}), (5,{"pwm":5})]:
            self.entries = deepcopy(original)
            self.entries[index].update(fields)
            with self.subTest(index=index,fields=fields), self.assertRaises(ValueError):
                self.issue()
