#!/usr/bin/env python3
"""구독 진단 — "저쪽은 쏘는데 나는 못 받는다" 를 잡아낸다.

ROS 2 에서 메시지가 안 오는 이유는 세 가지인데, 셋 다 **에러가 안 난다.**

  1. 아무도 발행하지 않는다        → 상대 노드가 안 떠 있다
  2. 메시지 타입이 다르다          → Float32 로 쏘는데 Int32 로 구독
  3. QoS 가 안 맞는다              → BEST_EFFORT 발행 vs RELIABLE 구독

2번과 3번은 토픽 이름이 똑같아서 `ros2 topic list` 로는 정상으로 보인다.
`ros2 topic echo` 로는 보이는데 노드만 못 받는 상황도 이것이다.

팀마다 PC 가 달라 "나는 되는데 너는 안 된다" 로 나타난다.
그래서 조용히 두지 않고 주기적으로 확인해 한 번씩 크게 찍는다.
"""


def _fmt_node(info):
    ns = (getattr(info, "node_namespace", "") or "").rstrip("/")
    name = getattr(info, "node_name", "?")
    return (ns + "/" + name) if ns else "/" + name


def _reliability_name(qos):
    """QoS reliability 를 사람이 읽는 이름으로. rclpy 버전차를 흡수한다."""
    try:
        r = qos.reliability
    except Exception:
        return None
    name = getattr(r, "name", None)
    if name:
        return name.upper()
    # 정수로 오는 구현도 있다: 1 = RELIABLE, 2 = BEST_EFFORT
    return {1: "RELIABLE", 2: "BEST_EFFORT"}.get(int(r), str(r))


def check_subscriptions(node, specs, seen, reliable=True):
    """구독 토픽들을 점검하고 처음 발견한 문제만 로그로 남긴다.

    node     : rclpy Node
    specs    : [(토픽, 기대 타입 문자열, 라벨)] 예) ("/lidar_drive",
               "std_msgs/msg/Float32", "lidar 구동")
    seen     : 중복 로그 방지용 set (호출자가 들고 있는다)
    reliable : 우리 구독이 RELIABLE 인가 (기본 True — create_subscription 기본값)

    발행자가 아직 없는 것은 정상일 수 있으므로(팀 노드가 늦게 뜬다) 조용히 넘긴다.
    """
    for topic, want_type, label in specs:
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception:
            continue
        if not infos:
            continue

        for info in infos:
            who = _fmt_node(info)

            # ---- 타입 불일치 ----
            got_type = getattr(info, "topic_type", "") or ""
            if want_type and got_type and got_type != want_type:
                key = ("type", topic, who, got_type)
                if key not in seen:
                    seen.add(key)
                    node.get_logger().error(
                        "%s: '%s' 메시지 타입이 다르다 — %s 는 %s 로 쏘는데 "
                        "우리는 %s 로 구독한다. 이러면 한 개도 안 들어온다. "
                        "(에러가 안 나서 토픽 목록에는 정상으로 보인다)"
                        % (label, topic, who, got_type, want_type))

            # ---- QoS 불일치 ----
            rel = _reliability_name(getattr(info, "qos_profile", None))
            if reliable and rel == "BEST_EFFORT":
                key = ("qos", topic, who)
                if key not in seen:
                    seen.add(key)
                    node.get_logger().error(
                        "%s: '%s' QoS 가 안 맞는다 — %s 는 BEST_EFFORT 로 "
                        "쏘는데 우리는 RELIABLE 로 구독한다. 연결 자체가 안 맺어져 "
                        "한 개도 안 들어온다. 발행 쪽을 RELIABLE 로 바꾸거나 "
                        "우리 구독을 BEST_EFFORT 로 내려야 한다."
                        % (label, topic, who))
