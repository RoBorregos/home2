#!/usr/bin/env python3
"""Offline A/B: rules (current FSM policy) vs LLM supervisor on simulated worlds.

Both policies see exactly the same worlds (same seeds). The LLM's real latency
is charged to the simulated task clock.

    python agentic_ab_benchmark.py --episodes 20 \
        --base-url http://localhost:11434/v1 --model qwen3 --out results.json
"""

import argparse
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_manager.agentic.executor import SimClock, TaskExecutor  # noqa: E402
from task_manager.agentic.sim_skills import make_sim_skills, sample_world  # noqa: E402
from task_manager.agentic.spec import load_spec  # noqa: E402
from task_manager.agentic.supervisor import make_supervisor  # noqa: E402

DEFAULT_SPEC = os.path.join(
    os.path.dirname(__file__), "..", "..", "config", "agentic", "restaurant_lite.yaml"
)


def percentile(values, p):
    if not values:
        return 0.0
    values = sorted(values)
    k = min(len(values) - 1, max(0, round(p / 100 * (len(values) - 1))))
    return values[k]


class ChargedLatency:
    """Charge a fixed per-decision latency to the sim clock instead of the real
    one (e.g. the expected Orin latency when benchmarking on a desktop CPU).
    The real latency is kept for the report."""

    def __init__(self, inner, charge_s):
        self.inner = inner
        self.name = inner.name
        self.charge_s = charge_s
        self.real = []

    def decide(self, ctx):
        decision = self.inner.decide(ctx)
        if decision.source != "rules":
            self.real.append(decision.latency_s)
            decision.latency_s = self.charge_s
        return decision


def run_episode(spec, seed, policy, llm_kwargs, latency_charge=None):
    world = sample_world(seed)
    clock = SimClock()
    supervisor = make_supervisor(
        policy,
        permanent_errors=spec.permanent_errors,
        **(llm_kwargs if policy == "llm" else {}),
    )
    if policy == "llm" and latency_charge is not None:
        supervisor = ChargedLatency(supervisor, latency_charge)
    res = TaskExecutor(
        spec, make_sim_skills(world, clock), supervisor, clock=clock, log=lambda _: None
    ).run()
    return {
        "seed": seed,
        "policy": policy,
        "world": world.objects,
        "points": res.points,
        "max_points": res.max_points,
        "elapsed_s": res.elapsed_s,
        "timed_out": res.timed_out,
        "aborted": res.aborted,
        "fallbacks": res.fallbacks_run,
        "attempts": res.attempts,
        "decisions": [
            {
                "action": d.action,
                "source": d.source,
                "reason": d.reason,
                "say": d.say,
                "latency_s": round(d.latency_s, 3),
            }
            for d in res.decisions
        ],
        "invalid": res.invalid_decisions,
        "latencies": supervisor.real
        if isinstance(supervisor, ChargedLatency)
        else res.llm_latencies,
        "utterances": res.utterances,
        "events": res.events,
    }


def summarize(episodes):
    lat = [x for e in episodes for x in e["latencies"]]
    ratio = [e["points"] / e["max_points"] for e in episodes if e["max_points"]]
    return {
        "episodes": len(episodes),
        "points_mean": statistics.mean(e["points"] for e in episodes),
        "score_ratio_mean": statistics.mean(ratio) if ratio else 0.0,
        "elapsed_mean_s": statistics.mean(e["elapsed_s"] for e in episodes),
        "timeouts": sum(e["timed_out"] for e in episodes),
        "aborted": sum(e["aborted"] for e in episodes),
        "fallbacks_mean": statistics.mean(e["fallbacks"] for e in episodes),
        "attempts_mean": statistics.mean(e["attempts"] for e in episodes),
        "decisions": sum(len(e["decisions"]) for e in episodes),
        "invalid_decisions": sum(e["invalid"] for e in episodes),
        "latency_p50_s": percentile(lat, 50),
        "latency_p95_s": percentile(lat, 95),
        "utterances": sum(len(e["utterances"]) for e in episodes),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", default=DEFAULT_SPEC)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed0", type=int, default=0)
    parser.add_argument("--policies", default="rules,llm")
    parser.add_argument("--base-url", default="http://localhost:11434/v1")
    parser.add_argument("--model", default="qwen3")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--retry-cap-factor", type=float, default=1.0)
    parser.add_argument(
        "--latency-charge",
        type=float,
        default=None,
        help="seconds charged to the sim clock per LLM decision (default: real latency)",
    )
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    spec = load_spec(args.spec)
    llm_kwargs = {
        "base_url": args.base_url,
        "model": args.model,
        "persona": spec.persona,
        "rules": spec.rules,
        "timeout_s": args.timeout,
        "retry_cap_factor": args.retry_cap_factor,
    }

    all_episodes, summary = [], {}
    for policy in args.policies.split(","):
        episodes = []
        for seed in range(args.seed0, args.seed0 + args.episodes):
            ep = run_episode(spec, seed, policy, llm_kwargs, args.latency_charge)
            episodes.append(ep)
            print(
                f"{policy:5s} seed={seed:3d} points={ep['points']:5.0f}/{ep['max_points']:.0f} "
                f"t={ep['elapsed_s']:5.0f}s fallbacks={ep['fallbacks']} invalid={ep['invalid']}",
                flush=True,
            )
        summary[policy] = summarize(episodes)
        all_episodes += episodes

    print(json.dumps(summary, indent=2))
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(
                {"summary": summary, "args": vars(args), "episodes": all_episodes}, f, indent=1
            )


if __name__ == "__main__":
    main()
