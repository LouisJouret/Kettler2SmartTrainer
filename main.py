import asyncio
from server import FTMS_server
from kettler import KettlerRacer9
import request_handler


async def run():
    kettler = KettlerRacer9()
    server = FTMS_server()

    await server.add_gatt(server.gatt)
    await server.start()
    try:
        while True:
            request = server.get_characteristic(server.control_point_char_uuid)
            request_handler.case.get(
                request.value[0], request_handler.default)()

            kettler.read()
            server.push_bike_metrics(kettler.cadence, kettler.power)
            await asyncio.sleep(2)

    except asyncio.CancelledError:
        kettler.direct_write("RS")
        _ = kettler.port.readline().decode("utf-8").strip()
        await server.stop()


asyncio.run(run())
