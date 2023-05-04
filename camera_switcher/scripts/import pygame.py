from evdev import InputDevice, ecodes, ff, util

# Change the event number to match your controller's event number
dev = InputDevice('/dev/input/event9')

# Create a rumble effect
rumble = ff.Rumble(strong_magnitude=0xffff, weak_magnitude=0xffff)

# Upload the effect to the device
effect_id = dev.upload_effect(rumble)

# Start the effect
dev.write(ecodes.EV_FF, effect_id, 1)

# Wait for the effect to finish
util.ff_effect_type(effect_id)[1].wait()

# Stop the effect
dev.write(ecodes.EV_FF, effect_id, 0)

# Unupload the effect from the device
dev.erase_effect(effect_id)
