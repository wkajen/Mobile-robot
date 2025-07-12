import lgpio
import time

chip = 4 
gpio_pin = 22  # GPIO numer (BCM)

h = lgpio.gpiochip_open(chip)
lgpio.gpio_claim_output(h, gpio_pin, 0)

try:
    while True:
        lgpio.gpio_write(h, gpio_pin, 1)
        time.sleep(1)
        # print(" LED on")
        lgpio.gpio_write(h, gpio_pin, 0)
        time.sleep(1)
except KeyboardInterrupt:
    lgpio.gpio_write(h, gpio_pin, 0)
    # print("Zamykam...")
finally:
    lgpio.gpiochip_close(h)
