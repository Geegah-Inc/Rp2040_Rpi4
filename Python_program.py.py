import PySimpleGUI as sg
import time
import serial
#%%
# open microcontroller serial port
#ser = serial.Serial('/dev/tty.usbmodem14401', 115200)
ser = serial.Serial('/dev/ttyACM0', 115200)
 
#%%
font_spec = 'Courier 24 bold'
heading_color = '#2FB8AD'
layout = [  [sg.Text('Mode selector'),
             sg.Radio('Mode 0', "radio1", default=True, key='radio1_1', enable_events=False),
             sg.Radio('Mode 1', "radio1", key='radio1_2', enable_events=False),
             sg.Radio('Mode 2', "radio1", key='radio1_3', enable_events=False)],
            [sg.Text('ADC selector'),
             sg.Radio('ADC 1', "radio2", default=True, key='radio2_1', enable_events=False),
             sg.Radio('ADC 2', "radio2", key='radio2_2', enable_events=False)],
            [sg.Text('State 4 Timing (ns)'),
             sg.Slider(range=(25,450), default_value=125, resolution=5, size=(22,15), key='slider2', 
             orientation='horizontal', font=('Helvetica', 12),enable_events=False)],
            [sg.Button('Send', font='Helvetica 12')],
            #
            #
            [sg.Text('Serial data from RP2040', background_color=heading_color)],
            [sg.Multiline('', size=(70,50), key='console',
               autoscroll=True, enable_events=False)],
            #
            [sg.Button('Exit', font='Helvetica 12')]
         ]

# change the colors in any way you like.
sg.SetOptions(background_color='#9FB8AD',
       text_element_background_color='#9FB8AD',
       element_background_color='#475841',#'#9FB8AD',
       scrollbar_color=None,
       input_elements_background_color='#9FB8AD',#'#F7F3EC',
       progress_meter_color = ('green', 'blue'),
       button_color=('white','#475841'),
       )

# Create the Window
window = sg.Window('Geegah Control Interface', layout, location=(0,0), 
                    return_keyboard_events=True, use_default_focus=True,
                    element_justification='c', size=(715,207), finalize=True)

# Event Loop to process "events" 
# event is set by window.read
event = 0
#
#  button state machine variables
button_on = 0
button_which = '0'
#
#
while True:
     
    # time out paramenter makes the system non-blocking
    # If there is no event the call returns event  '__TIMEOUT__'
    event, values = window.read(timeout=20) # timeout=10
    #
    #print(event)  # for debugging
    # if user closes window using windows 'x' or clicks 'Exit' button  
    if event == sg.WIN_CLOSED or event == 'Exit': # 
        break
    #
    if event=='Send':
        if (values['radio1_1'])==True:
            mode='0'
        elif (values['radio1_2'])==True:
            mode='1'
        else:
            mode='2'
            
        timing = str(int(values['slider2']))
        
        if (values['radio2_1'])==True:
            adc='1'
        else:
            adc='2'
        print(timing)
        print(type(timing))
        packet = str(mode+'.'+timing+'.'+adc+'\n')
            
        print(packet)
        ser.write(packet.encode())

    # character loopback from PIC
    while ser.in_waiting > 0:
       #serial_chars = (ser.read().decode('utf-8'));
       #window['console'].update(serial_chars+'\n', append=True)
       pic_char = chr(ser.read(size=1)[0]) 
       if (pic_char) == '\n' :
          window['console'].update('\n', append=True)
       else :
          window['console'].update((pic_char), append=True)
     
# close port and Bail out
ser.close()             
window.close()
