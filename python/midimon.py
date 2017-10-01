import mido

print('Listening for MIDI input...')

with mido.open_input() as inport:
    for msg in inport:
        print(msg)
