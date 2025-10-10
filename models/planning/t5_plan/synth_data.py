import json, random, uuid, argparse, os
from pathlib import Path

ROOMS = {
    "charging_dock": (1.5,-0.7), "conveyor_belt": (1.3,-0.4),
    "station_a": (0.8,0.9), "station_b": (-0.6,0.4), "door": (0.0,-1.2)
}

TEMPLATES = [
    "go to the {dst}",
    "navigate to {dst} then face {face} and wait {secs} seconds",
    "head to {dst} at slow speed",
    "go to {dst} avoiding wet floor",
    "move to {dst}, stop",
    "go to {dst} then face {angle} degrees",
]

def make_example():
    dst = random.choice(list(ROOMS))
    face = random.choice(list(ROOMS))
    secs = random.choice([2,3,5,10])
    angle = random.choice([0,90,180,-90])
    tpl = random.choice(TEMPLATES)
    instr = tpl.format(dst=dst, face=face, secs=secs, angle=angle)

    speed_limit = 0.3 if "slow" in instr else 0.5
    avoid = ["wet_floor"] if "avoiding" in instr else []

    steps = [{"op":"goto","x":ROOMS[dst][0],"y":ROOMS[dst][1]}]
    if "face" in instr:
        if "{angle}" in tpl:
            steps.append({"op":"face","theta_deg": angle})
        else:
            steps.append({"op":"face","theta_deg": random.choice([0,90,180,-90])})
    if "wait" in instr:
        steps.append({"op":"wait","seconds":secs})
    if "stop" in instr or random.random() < 0.7:
        steps.append({"op":"stop"})

    goal_lib = "; ".join([f"{k}=({v[0]},{v[1]})" for k,v in ROOMS.items()])
    input_text = (
        f"INSTRUCTION:\n{instr}\n\n"
        f"ROBOT_STATE:\npose=(x=0.0,y=0.0,theta=0deg)\n\n"
        f"GOAL_LIBRARY:\n{goal_lib}\n\n"
        f"CONSTRAINTS:\nspeed_limit={speed_limit}, avoid={avoid}\n\n"
        f"OUTPUT JSON:\n"
    )

    target_text = json.dumps({
        "plan_id": str(uuid.uuid4()),
        "steps": steps,
        "constraints": {"avoid": avoid, "speed_limit": speed_limit}
    }, ensure_ascii=False)

    return {"input_text": input_text, "target_text": target_text}

def dump(n: int, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for _ in range(n):
            ex = make_example()
            f.write(json.dumps(ex)+"\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-train", type=int, default=8000)
    parser.add_argument("--n-val", type=int, default=1000)
    parser.add_argument("--out-train", type=Path, default=Path("../../data/train.jsonl"))
    parser.add_argument("--out-val", type=Path, default=Path("../../data/val.jsonl"))
    args = parser.parse_args()

    dump(args.n_train, Path(__file__).resolve().parent.joinpath(args.out-train).resolve())
    dump(args.n_val,   Path(__file__).resolve().parent.joinpath(args.out-val).resolve())
