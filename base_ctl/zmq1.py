import zmq
import json

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://10.8.0.90:4399")

# ✅ 改1：用 bytes 订阅，和他 b"control" 完全一致
socket.setsockopt(zmq.SUBSCRIBE, b"control")

print("[ZMQ] Listening on control...")

while True:
    try:
        topic, payload = socket.recv_multipart()

        # ✅ 改2：可选，严格确认 topic（避免别的topic混进来）
        if topic != b"control":
            continue

        # ✅ 改3：json.loads 可以直接吃 bytes，不一定要 decode（保留 decode 也行）
        msg = json.loads(payload)

        base_X = msg["base_X"]
        base_Y = msg["base_Y"]
        pitch  = msg["pitch"]
        cmd    = msg.get("command", "")

        print(f"[CONTROL] cmd={cmd} X={base_X:.3f}, Y={base_Y:.3f}, pitch={pitch:.3f}")

    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Error:", e)
