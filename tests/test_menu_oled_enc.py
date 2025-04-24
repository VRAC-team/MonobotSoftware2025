import gpiod
import datetime
import luma.core.interface.serial
import luma.oled.device
from luma.core.render import canvas

GPIO_ENC_A = 17
GPIO_ENC_B = 27
GPIO_ENC_PUSH = 22
GPIO_BUTTON = 23

oled_i2c = luma.core.interface.serial.i2c(port=1, address=0x3C)
oled = luma.oled.device.ssd1306(oled_i2c)

menu_items = ["Match", "Manette", "Tests"]
selected_index = 0

def render_menu():
    with canvas(oled) as draw:
        for i, item in enumerate(menu_items):
            y = i * 10
            prefix = ">" if i == selected_index else " "
            draw.text((0, y), f"{prefix} {item}", fill=255)

        draw.text((0, 33), "motor:..........", fill=255)
        draw.text((0, 43), "servo:..........", fill=255)
        draw.text((0, 53), "   io:..........", fill=255)

def scroll_up():
    global selected_index
    if selected_index > 0:
        selected_index -= 1
    render_menu()

def scroll_down():
    global selected_index
    if selected_index < len(menu_items)-1:
        selected_index += 1
    render_menu()

def watch_multiple_line_values():
    with gpiod.request_lines(
        "/dev/gpiochip0",
        consumer="test_encoder",
        config={
            (GPIO_ENC_A, GPIO_ENC_B): gpiod.LineSettings(
                direction=gpiod.line.Direction.INPUT,
                edge_detection=gpiod.line.Edge.RISING,
                bias=gpiod.line.Bias.PULL_UP,
                debounce_period=datetime.timedelta(milliseconds=5)
            ),
            (GPIO_ENC_PUSH, GPIO_BUTTON): gpiod.LineSettings(
                direction=gpiod.line.Direction.INPUT,
                edge_detection=gpiod.line.Edge.FALLING,
                bias=gpiod.line.Bias.PULL_UP,
                debounce_period=datetime.timedelta(milliseconds=10)
            )
        },
    ) as request:
        counter = 0

        while True:
            for event in request.read_edge_events():
                if event.line_offset == GPIO_ENC_A:
                    b = request.get_value(GPIO_ENC_B)
                    if b:
                        counter = counter-1
                        scroll_down()
                    else:
                        counter = counter+1
                        scroll_up()

                elif event.line_offset == GPIO_ENC_PUSH:
                    print("enc_push")
                elif event.line_offset == GPIO_BUTTON:
                    print(f"Selected: {menu_items[selected_index]}")

render_menu()
watch_multiple_line_values()
