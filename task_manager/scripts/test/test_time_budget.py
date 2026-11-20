#!/usr/bin/env python3

"""TimeBudget self-check with a fake clock: python3 test_time_budget.py (or pytest)."""

from task_manager.utils.time_budget import Priority, Step, TimeBudget


def test_time_budget():
    now = [0.0]
    chain = [
        Step("greet", 30, Priority.HIGH),
        Step("nav", 40, Priority.MUST),
        Step("chat", 20, Priority.LOW),
        Step("intro", 30, Priority.MUST),
    ]
    b = TimeBudget(120, chain, clock=lambda: now[0])

    assert b.enter("greet") == 0
    assert b.reserve(Priority.LOW) == 70  # nav + intro (chat is not more important than LOW)
    assert b.reserve(Priority.HIGH) == 70
    assert b.allows(Priority.HIGH, cost=50)  # 120 - 50 >= 70
    assert not b.allows(Priority.HIGH, cost=51)
    assert b.allows(Priority.MUST, cost=1000)  # MUST is never skipped
    assert not b.behind()

    now[0] = 60  # greeting dialog dragged on
    assert b.behind()  # 60s left < nav + chat + intro
    assert not b.allows(Priority.HIGH, cost=10)  # another retry would eat into nav + intro

    assert b.enter("nav") == 30  # 30s behind plan
    assert b.enter("nav") == 30  # repeated state keeps the cursor
    now[0] = 85
    b.enter("chat")
    assert not b.allows(Priority.LOW, cost=10)  # 35 - 10 < 30 reserved for intro
    assert b.allows(Priority.LOW, cost=5)

    # extra work (retries) also protects the steps as important as itself
    now[0] = 0
    b = TimeBudget(120, chain, clock=lambda: now[0])
    b.enter("greet")
    assert b.reserve(Priority.LOW, extra=True) == 90  # nav + chat + intro
    assert b.allows(Priority.LOW, cost=31) and not b.allows(Priority.LOW, cost=31, extra=True)
    assert b.affords("chat")  # 120 - 20 >= nav + intro
    now[0] = 40
    assert not b.affords("chat")  # 80 - 20 < 70: skip the whole LOW step
    assert b.affords("intro") and b.affords("unknown")  # MUST and unplanned steps always run


if __name__ == "__main__":
    test_time_budget()
    print("test_time_budget OK")
