import time
from wideboy import Interface

if __name__ == "__main__":
    interface = Interface()

    cell_count, battery_volts, low_voltage_cutoff = interface.get_battery_state()
    battery_volts /= 1000.0
    low_voltage_cutoff /= 1000.0
    volt_per_step = battery_volts / 400

    print('Battery Volts: {:f}'.format(battery_volts))
    print('Volts per Step: {:f}'.format(volt_per_step))

    done = False
    current_step = 0
    interface.set_motor_speeds(current_step, current_step)
    time.sleep(1.0)

    while not done:
        interface.set_motor_speeds(current_step, current_step)
        time.sleep(1.0)
        motor_rates = interface.get_motor_rates()
        print('step: {:d} volts: {:f} counts: [{:d},{:d}]'.format(current_step, current_step * volt_per_step, motor_rates[0], motor_rates[1]))
        done = motor_rates[0] > 0 and motor_rates[1] > 0
        current_step += 1

    interface.set_motor_speeds(0, 0)
