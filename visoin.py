import socket
import time

try:
    from maix import uart, pinmap
    
    print(f"当前系统可用串口: {uart.list_devices()}")
    
    # === 新增自动查找串口引脚代码，帮您排查 ===
    all_pins = pinmap.get_pins()
    print("--- 正在扫描系统支持的串口引脚映射 ---")
    for p in all_pins:
        try:
            funcs = pinmap.get_pin_functions(p)
            uart_funcs = [f for f in funcs if "UART" in f]
            if uart_funcs:
                print(f"引脚 {p} 可用作: {uart_funcs}")
        except:
            pass
    print("----------------------------------------")
    
    # 您指定了 A16 和 A17。根据底层日志，这通常是分配给 UART0 (/dev/ttyS0) 的
    try:
        pinmap.set_pin_function("A16", "UART0_TX")
        pinmap.set_pin_function("A17", "UART0_RX")
    except Exception as e:
        print(f"引脚映射提示: {e}")

    # 使用对应的串口 UART0 (/dev/ttyS0)
    UART_PORT = uart.UART("/dev/ttyS0", 115200)
    print("UART 串口初始化成功! (/dev/ttyS0, 映射 TX=A16, RX=A17)")
except Exception as e:
    print(f"UART 串口初始化失败: {e}")
    UART_PORT = None

# =========================================================
# UDP 透传设置
# =========================================================
UDP_IP = "255.255.255.255"  # 广播给上位机
UDP_PORT = 8080             # 上位机监听的端口
UDP_BIND_PORT = 8081        # Maixcam 监听上位机下发指令的端口

try:
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    udp_socket.bind(("0.0.0.0", UDP_BIND_PORT))
    udp_socket.setblocking(False)
    print(f"UDP 透传初始化成功! 将数据广播至 {UDP_IP}:{UDP_PORT}")
    print(f"同时监听 UDP 端口 {UDP_BIND_PORT} 接收上位机指令")
except Exception as e:
    print(f"UDP 初始化失败: {e}")
    udp_socket = None

def send_uart_command(target_name):
    """
    通信协议: AA 55 [转向信号] [颜色信号] [形状信号] [校验和]
    与 main.py 和 3519 的协议保持一致，供测试使用
    """
    if not UART_PORT:
        return

    if target_name is None or "_" not in target_name:
        return

    color_str, shape_str = target_name.split("_")
    if color_str not in ["red", "green"] or shape_str not in ["circle", "square"]:
        return

    val_color = 0x01 if color_str == "red" else 0x00
    val_turn = val_color # 红色右转，绿色左转
    val_shape = 0x01 if shape_str == "circle" else 0x00
    checksum = (val_turn + val_color + val_shape) & 0xFF

    packet = bytearray([0xAA, 0x55, val_turn, val_color, val_shape, checksum])
    
    try:
        UART_PORT.write(bytes(packet))
        print(f"[UART 发送给3519]: 动作={target_name} 报文={' '.join([f'{b:02X}' for b in packet])}")
    except Exception as e:
        print(f"[UART 发送异常]: {e}")

def main():
    print("双向通信透传脚本启动...")
    
    last_send_time = time.time()
    send_state = 1
    
    while True:
        # ============================================
        # 自动测试：每隔一秒交替发送 "1\n" 和 "2\n"
        # ============================================
        current_time = time.time()
        if current_time - last_send_time >= 1.0:
            last_send_time = current_time
            if UART_PORT:
                try:
                    if send_state == 1:
                        UART_PORT.write(b"1\n")
                        print("[自动测试] 自动发送: 1")
                        send_state = 2
                    else:
                        UART_PORT.write(b"2\n")
                        print("[自动测试] 自动发送: 2")
                        send_state = 1
                except Exception as e:
                    print(f"发送异常: {e}")

        # 1. 接收 3519 UART 数据并 UDP 转发给上位机
        if UART_PORT:
            try:
                in_data = UART_PORT.read()
                if in_data:
                    # 转给上位机
                    if udp_socket:
                        try:
                            udp_socket.sendto(bytes(in_data), (UDP_IP, UDP_PORT))
                        except Exception:
                            pass
                    
                    # 在终端打印，方便调试
                    try:
                        text = in_data.decode("utf-8", "ignore")
                        if text:
                            print(f"[收到 3519 文本]: {text.strip()}")
                    except:
                        pass
            except Exception:
                pass
        
        # 2. 接收上位机 UDP 数据，处理后发给 3519
        if udp_socket:
            try:
                udp_data, addr = udp_socket.recvfrom(1024)
                if udp_data:
                    text = udp_data.decode("utf-8", "ignore").strip()
                    print(f"[收到上位机 UDP 指令]: {text}")
                    # 如果上位机发的是 "red_circle" 等指令，自动封包成二进制发给3519测动作
                    if "_" in text and text.startswith(("red", "green")):
                        send_uart_command(text)
                    elif UART_PORT:
                        # 其他普通指令 (比如 "STOP", "L500R-500") 原封不动透传给 3519
                        UART_PORT.write(udp_data)
            except BlockingIOError:
                pass
            except Exception as e:
                pass

        time.sleep(0.01)

if __name__ == "__main__":
    main()
