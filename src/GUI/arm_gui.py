import PySimpleGUI as sg
import serial
import time


#USB Serial Communication
USBSerial = serial.Serial('COM5', 9600)
USBSerial.timeout = 1

def print_encoded(data):
    encoded_data = data.encode()
    print(' '.join(str(byte) for byte in encoded_data))


tab1 = [[sg.Text('Welcome to the Robotic Arm Interface! Read the following instructions to interact with the arm:')],

        [sg.Text(
            '\nFor G-Code Commands, click on the "G-Code Commands" tab, where the different G-Commands will be displayed.'), ],

        [sg.Text('Some G-Code Commands will require specified coordinates from the user'), ],

        [sg.Text('\nTo enter free-hand drawing mode, click on the free-hand drawing button below:')],

        [sg.Text('Free-hand mode:\t'),
         sg.Button('Free-hand drawing mode', key='free_mode'),
         sg.Button('Exit Free-hand mode', key='exit_mode', disabled=True)]]

tab2 = [
    [sg.Text('Enter G-Code commands and parameters')],

    [sg.Text('G0', size=(20, 1)),
     sg.Input(size=(10, 1), key='xpos', default_text="X ##", ),
     sg.Input(size=(10, 1), key='ypos', default_text="Y ##"),
     sg.Button('Apply G0 Command', key='g0', size=20)],

    [sg.Text('G1', size=(20, 1)),
     sg.Input(size=(10, 1), key='xpos', default_text="X ##", ),
     sg.Input(size=(10, 1), key='ypos', default_text="Y ##"),
     sg.Input(size=(10, 1), key='vel', default_text="V ##", ),
     sg.Button('Apply G1 Command', key='g1', size=20)],

    [sg.Text('G20', size=(20, 1)), sg.Button('Apply G20 Command', key='g20', size=20)],

    [sg.Text('G21', size=(20, 1)), sg.Button('Apply G21 Command', key='g21', size=20)],

    [sg.Text('G90', size=(20, 1)), sg.Button('Apply G90 Command', key='g90', size=20)],

    [sg.Text('G91', size=(20, 1)), sg.Button('Apply G91 Command', key='g91', size=20)],

    [sg.Text('M2', size=(20, 1)), sg.Button('Apply M2 Command', key='m2', size=20)],

    [sg.Text('M6', size=(20, 1)), sg.Button('Apply M6 Command', key='m6', size=20)],

    [sg.Text('M72', size=(20, 1)), sg.Button('Apply M72 Command', key='m72', size=20)]]

# Putting everything together into a single window.
layout = [
    [sg.TabGroup([[sg.Tab('Main Interface', tab1), sg.Tab('G-Code Commands', tab2)]])],
    [sg.Output(size=(80, 10), font='Verdana 10')],
    [sg.Button('Exit', key='exit')]]

window = sg.Window('Robot Arm Interface', layout, location=(200, 0), return_keyboard_events=True,
                   use_default_focus=False)

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'exit':  # if user closes window or clicks cancel
        break
    if event == 'free_mode':
        print("Entering Free-hand drawing mode. Use WASD Keys to control")
        window['exit_mode'].update(disabled=False)
        window['free_mode'].update(disabled=True)
        window.refresh()
        q = 'q'
        USBSerial.write(q.encode(errors='ignore'))

        while event != 'exit_mode':
            event, values = window.read()
            window.refresh()
            if len(event) == 1:
                print(event)
                serial_data = event + "\n"
                USBSerial.write(serial_data.encode(errors='ignore'))
                print_encoded(serial_data)
                time.sleep(0.08)

            elif event == 'exit':
                window.close()



        print("Exiting Free-hand drawing mode")
        serial_data = 'q\n'
        USBSerial.write(serial_data.encode(errors='ignore'))
        window['exit_mode'].update(disabled=True)
        window['free_mode'].update(disabled=False)

    elif event == 'g0':

        serial_data = "G00" + values['xpos'] + values['ypos'] + "\n"
        # print(serial_data)
        print("Executing G0 Command")
        USBSerial.write(serial_data.encode(errors='ignore'))
        print_encoded(serial_data)

    elif event == 'g1':

        serial_data = "G01" + values['xpos'] + values['ypos'] + values['vel'] + "\n"
        # print(serial_data)
        print("Executing G1 Command")
        USBSerial.write(serial_data.encode(errors='ignore'))
        print_encoded(serial_data)

    elif event == 'g20':
        serial_data = "G20\n"
        USBSerial.write(serial_data.encode(errors='ignore'))
        print("Executing G20 Command")
        print_encoded(serial_data)

    elif event == 'g21':
        serial_data = 'G21\n'
        USBSerial.write(serial_data.encode(errors='ignore'))
        print("Executing G21 Command")
        print_encoded(serial_data)

    elif event == 'g90':
        serial_data = 'G90\n'
        USBSerial.write(serial_data.encode(errors='ignore'))
        print("Executing G90 Command")
        print_encoded(serial_data)

    elif event == 'g91':
        serial_data = 'G91\n'
        USBSerial.write(serial_data.encode(errors='ignore'))
        print("Executing G91 Command")
        print_encoded(serial_data)

    elif event == 'm2':
        serial_data = 'M02\n'
        USBSerial.write(serial_data.encode(errors='ignore'))
        print("Executing M2 Command")
        print_encoded(serial_data)

    elif event == 'm6':
        serial_data = 'M06\n'
        print("Executing M6 Command")
        USBSerial.write(serial_data.encode(errors='ignore'))
        print_encoded(serial_data)

    elif event == 'm72':
        serial_data = 'M72\n'
        print("Executing M72 Command")
        USBSerial.write(serial_data.encode(errors='ignore'))
        print_encoded(serial_data)

    usb_data = USBSerial.readline(10)
    print(usb_data)
    window.refresh()

window.close()