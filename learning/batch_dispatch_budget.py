"""모든 배치 실행 진입점이 공유하는 계획별 영속 명령 예산.

예약은 실행 전에 확정하며 실패·미시작도 반환하지 않는다. 출력 디렉터리나
모델 trial ID를 바꿔 같은 계획의 예산을 다시 사용할 수 없다.
"""
from pathlib import Path
import sqlite3

from learning.batch_campaign import digest, require, validate_plan


def reserve(repository, plan, condition, request_id):
    checked = validate_plan(plan)
    require(condition in checked['ready'], 'dispatch_budget_condition_not_in_plan')
    require(isinstance(request_id, str) and bool(request_id), 'dispatch_budget_request_required')
    root = Path(repository).resolve()/'.local/batch-dispatch-budgets'
    root.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(root/(digest(plan)+'.sqlite'), timeout=2)
    try:
        db.execute('PRAGMA synchronous=FULL')
        db.execute('CREATE TABLE IF NOT EXISTS reservations '
                   '(request_id TEXT PRIMARY KEY, condition_id TEXT NOT NULL, phase TEXT NOT NULL)')
        db.execute('BEGIN IMMEDIATE')
        require(db.execute('SELECT 1 FROM reservations WHERE request_id=?',
                           (request_id,)).fetchone() is None, 'dispatch_budget_request_consumed')
        total = db.execute('SELECT COUNT(*) FROM reservations').fetchone()[0]
        count = db.execute('SELECT COUNT(*) FROM reservations WHERE condition_id=?',
                           (condition['id'],)).fetchone()[0]
        require(count < condition['maxAttempts'], 'dispatch_budget_condition_exhausted')
        budget = plan['budgets']
        require(total < min(budget['maxPhysicalCommands'], budget['maxAttempts']),
                'dispatch_budget_plan_exhausted')
        if condition['phase'] == 'preparation':
            prepared = db.execute("SELECT COUNT(*) FROM reservations WHERE phase='preparation'").fetchone()[0]
            require(prepared < budget['maxPreparationCommands'], 'dispatch_budget_preparation_exhausted')
        db.execute('INSERT INTO reservations VALUES (?,?,?)',
                   (request_id, condition['id'], condition['phase']))
        db.commit()
    finally:
        db.close()
