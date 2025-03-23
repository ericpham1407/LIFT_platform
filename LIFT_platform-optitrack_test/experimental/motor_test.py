import asyncio
import time
import math

import moteus
from moteus.moteus import Result
import moteus.moteus
import moteus.multiplex

ID_fl = 1
ID_fr = 3

async def main():
    query_resuolution = moteus.moteus.QueryResolution()
    query_resuolution.q_current = moteus.multiplex.F32 # include current in default query response
    query_resuolution.d_current = moteus.multiplex.F32 # include current in default query response
    c_fl = moteus.Controller(query_resolution=query_resuolution, id = ID_fl)
    c_fr = moteus.Controller(query_resolution=query_resuolution, id = ID_fr)
    stream_fl = moteus.Stream(c_fl)
    await stream_fl.command(b'conf set servopos.position_min nan')
    await stream_fl.command(b'conf set servopos.position_max nan')
    stream_fr = moteus.Stream(c_fr)
    await stream_fr.command(b'conf set servopos.position_min nan')
    await stream_fr.command(b'conf set servopos.position_max nan')
    # Clear any faults.
    await c_fl.set_stop()
    await c_fl.set_rezero()
    await c_fr.set_stop()
    await c_fr.set_rezero()
    # Flip sign of left side motos
    await stream_fl.command(b'conf set motor_position.output.sign -1')

    # var_name = await stream.command(b'conf get servos...') # how to read the configurable variables

    while True:
        # Our command speed will alternate
        # every second.
        current_command = 1 if (round(time.time()) % 2) else -1

        # The acceleration and velocity limit could be configured as
        # `servo.default_accel_limit` and
        # `servo.default_velocity_limit`.  We will override those
        # configurations here on a per-command basis to ensure that
        # the limits are always used regardless of config.
        results_fl: Result = await c_fl.set_position(
            position=math.nan,
            velocity=current_command,
            accel_limit=10.0,
            velocity_limit=3.0,
            query=True,
        )
        results_fr: Result = await c_fr.set_position(
            position=math.nan,
            velocity=current_command,
            accel_limit=10.0,
            velocity_limit=3.0,
            query=True,
        )

        print("Results FL:", results_fl)
        print("Results FR:", results_fr)
        # print(results)
        #print(dir(results))
        #print(type(results))
        #print(results.id)
        #print(results.bus)
        #print(results.values)

        await asyncio.sleep(0.02)

async def stop():
    transport = moteus.Fdcanusb()
    c_fl = moteus.Controller(id = ID_fl)
    c_fr = moteus.Controller(id = ID_fr)
    await transport.cycle([
        c_fl.make_stop()
    ])
    await transport.cycle([
        c_fr.make_stop()
    ])

try:
    asyncio.run(main())
except KeyboardInterrupt:
    asyncio.run(stop())
