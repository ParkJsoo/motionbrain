#!/usr/bin/env python3
"""Inspect a bounded-action journal or explicitly append verified recovery evidence.

No camera, ARM, motor command, upload, retry, or training. Existing journal only.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from learning.action_journal import BoundedActionJournal, ResolutionReader, digest, prepare_resolution, read_journal
from learning.artifacts import ArtifactStore, _decode_json


def private_existing(path):
    path=path.resolve()
    if not path.is_relative_to((ROOT/'.local').resolve()) or not path.is_file():
        raise ValueError('existing_private_journal_required')
    return path


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--journal',type=Path,required=True)
    modes=ap.add_subparsers(dest='mode',required=True)
    status=modes.add_parser('status',help='read-only offline readiness; no schema migration')
    status.add_argument('--command-id')
    recover=modes.add_parser('recover',help='explicitly retire interrupted proposals, preserving before/after history')
    recover.add_argument('--confirm-snapshot-sha256',required=True)
    prepare=modes.add_parser('prepare',help='GET-only terminal/OFF evidence for human review; journal remains unchanged')
    prepare.add_argument('--command-id',required=True)
    prepare.add_argument('--controller-url',required=True)
    prepare.add_argument('--output-dir',type=Path,required=True)
    resolve=modes.add_parser('resolve',help='append reviewed resolution after fresh GET evidence; never overwrite original outcome')
    resolve.add_argument('--evidence',type=Path,required=True)
    resolve.add_argument('--confirm-sha256',required=True)
    resolve.add_argument('--controller-url',required=True)
    args=ap.parse_args(argv)
    path=private_existing(args.journal)
    if args.mode=='status':
        result=read_journal(path,command_id=args.command_id)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if result['ready'] else 2
    if args.mode=='prepare':
        if not args.output_dir.resolve().is_relative_to((ROOT/'.local').resolve()):
            raise ValueError('private_evidence_output_required')
        store=ArtifactStore(args.output_dir)
        evidence=prepare_resolution(path,args.command_id,ResolutionReader(args.controller_url))
        store.write_json('resolution-evidence.json',evidence)
        result=dict(mode='prepared_only',confirmedSha256=digest(evidence),journalChanged=False,
                    permitFutureNewCommandsOnly=True,trainingEligible=False,originalOutcomeChanged=False)
        store.write_json('review.json',result)
    else:
        # Validate the explicit confirmation before opening a writable journal.
        if args.mode=='recover':
            check=read_journal(path)
            if check['snapshotSha256']!=args.confirm_snapshot_sha256:
                raise ValueError('exact_recovery_confirmation_required')
        else:
            evidence=_decode_json(private_existing(args.evidence).read_bytes())
            if digest(evidence)!=args.confirm_sha256:
                raise ValueError('exact_resolution_confirmation_required')
        journal=BoundedActionJournal(path,10000)
        try:
            if args.mode=='recover':
                result=dict(recoveredIds=journal.recover_interrupted(args.confirm_snapshot_sha256),
                            readiness=journal.readiness(),hardwareCalls=0)
            else:
                result=journal.resolve(evidence,args.confirm_sha256,ResolutionReader(args.controller_url))
        finally:journal.close()
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except KeyboardInterrupt:raise SystemExit(130)
    except Exception:
        print(json.dumps(dict(error='journal_management_failed_preserve_original_and_inspect',automaticRetry=False)),file=sys.stderr)
        raise SystemExit(2)
