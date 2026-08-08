#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import rosbag
import rospy
from rosgraph_msgs.msg import Clock
from roslib.message import get_message_class


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ROS1 bag을 Python으로 읽어 토픽과 /clock을 재발행합니다."
    )
    parser.add_argument("bag", type=Path, help="재생할 ROS1 .bag 파일")
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="재생 속도. 기본값: 1.0",
    )
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="RViz 준비 Enter 대기 없이 즉시 재생합니다.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="재생할 bag 시간(초). 생략하면 전체를 재생합니다.",
    )
    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("--rate는 0보다 커야 합니다.")
    if args.duration is not None and args.duration <= 0:
        raise ValueError("--duration은 0보다 커야 합니다.")

    bag_path = args.bag.expanduser().resolve()

    if not bag_path.is_file():
        raise FileNotFoundError(f"bag 파일을 찾을 수 없습니다: {bag_path}")

    rospy.init_node("python_rosbag_player", anonymous=True)
    rospy.set_param("/use_sim_time", True)

    clock_pub = rospy.Publisher("/clock", Clock, queue_size=100)

    with rosbag.Bag(str(bag_path), "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        publishers = {}

        for topic, info in topic_info.items():
            # /clock은 이 프로그램에서 직접 생성합니다.
            if topic == "/clock":
                continue

            message_class = get_message_class(info.msg_type)

            if message_class is None:
                rospy.logwarn(
                    "메시지 타입을 불러오지 못해 건너뜁니다: %s [%s]",
                    topic,
                    info.msg_type,
                )
                continue

            publishers[topic] = rospy.Publisher(
                topic,
                message_class,
                queue_size=100,
                latch=(topic == "/tf_static"),
            )

        print(f"\nbag: {bag_path}")
        print(f"messages: {bag.get_message_count()}")
        print("publish topics:")

        for topic in publishers:
            print(f"  {topic}")

        if not args.no_wait:
            print("\nRViz 설정을 마친 후 Enter를 누르세요.")
            input()

        # RViz가 publisher 연결을 완료할 시간을 줍니다.
        time.sleep(1.0)

        first_bag_time = None
        wall_start = None

        for topic, message, stamp in bag.read_messages(
            topics=list(publishers.keys())
        ):
            if rospy.is_shutdown():
                break

            stamp_sec = stamp.to_sec()

            if first_bag_time is None:
                first_bag_time = stamp_sec
                wall_start = time.monotonic()

            if args.duration is not None and stamp_sec - first_bag_time > args.duration:
                break

            target_elapsed = (stamp_sec - first_bag_time) / args.rate

            # 메시지 사이에도 /clock을 계속 진행시킵니다.
            while not rospy.is_shutdown():
                wall_elapsed = time.monotonic() - wall_start
                remaining = target_elapsed - wall_elapsed

                if remaining <= 0:
                    break

                simulated_time = first_bag_time + wall_elapsed * args.rate
                clock_pub.publish(
                    Clock(clock=rospy.Time.from_sec(simulated_time))
                )

                time.sleep(min(0.01, remaining))

            clock_pub.publish(Clock(clock=stamp))
            publishers[topic].publish(message)

    print("\n재생이 끝났습니다.")


if __name__ == "__main__":
    main()
