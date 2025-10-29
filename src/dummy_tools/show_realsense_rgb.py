#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import argparse

import cv2
import pyrealsense2 as rs


def main():
    parser = argparse.ArgumentParser(description='实时显示 RealSense RGB 图像')
    parser.add_argument('--width', type=int, default=1920, help='RGB 图像宽度')
    parser.add_argument('--height', type=int, default=1080, help='RGB 图像高度')
    parser.add_argument('--fps', type=int, default=30, help='RGB 帧率')
    parser.add_argument('--device', type=str, default=None, help='指定设备序列号(可选)')
    args = parser.parse_args()

    pipeline = None
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        if args.device:
            config.enable_device(args.device)
        config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

        profile = pipeline.start(config)
        print('RealSense RGB 流已启动')
        color_sensor = profile.get_device().first_color_sensor()
        if color_sensor:
            try:
                auto_exp = color_sensor.get_option(rs.option.enable_auto_exposure)
                auto_white = color_sensor.get_option(rs.option.enable_auto_white_balance)
                print(f'- 自动曝光: {"开启" if auto_exp else "关闭"}')
                print(f'- 自动白平衡: {"开启" if auto_white else "关闭"}')
            except Exception:
                pass

        win_name = 'RealSense RGB (按 q 退出)'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 960, 540)

        # 预热几帧，提高曝光与白平衡稳定性
        for _ in range(5):
            pipeline.wait_for_frames()
            time.sleep(0.02)

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = cv2.flip(cv2.cvtColor(
                cv2.imdecode(
                    cv2.imencode('.bmp', cv2.UMat(color_frame.get_data()).get())[1],
                    cv2.IMREAD_COLOR
                ),
                cv2.COLOR_BGR2BGR
            ), 1) if False else None  # 保留占位, 实际直接从缓冲区复制

            # 直接从帧数据复制到 numpy 数组（更高效）
            color_image = None
            try:
                import numpy as np
                color_image = np.asanyarray(color_frame.get_data())
            except Exception:
                pass

            if color_image is None:
                continue

            cv2.imshow(win_name, color_image)
            print(f"color_image.shape: {color_image.shape}")
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()
        print('已退出显示')
        return 0

    except KeyboardInterrupt:
        print('\n已中断')
        return 0
    except Exception as e:
        print(f'发生错误: {e}')
        return 1
    finally:
        try:
            if pipeline is not None:
                pipeline.stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main()) 