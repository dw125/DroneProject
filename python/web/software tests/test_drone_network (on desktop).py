import asyncio
import time
import subprocess
from websockets.asyncio.client import connect

testing = True

#put this into config file?
remoteserver = "ws://130.95.13.173:39425"
localserver = "ws://localhost:39425"

globallock = asyncio.Lock()
lastcontrolled = time.time()

# State should be "Stopped", "Running", "Waiting" or "Unknown"
state = "Stopped";

async def launch(height, wait_time):
	global state
	async with globallock:
		state = "Running"
		print("Running")
		print("Height (m):" + height)
		print("Wait time (s):" + wait_time)
	return

async def stop():
	global state
	async with globallock:
		state = "Stopped"
	return

async def handleMessage(message, websocket):
	data = message.split("/");
	if len(data) > 0:
		print(data)
		if data[0] == "LAUNCH":
			if len(data) >= 3:
				await launch(data[1], data[2])
		if data[0] == "STOP":
			await stop()
	await websocket.send("HEARTBEAT/DRONE/" + state)

async def websockethandler():
	global lastcontrolled
	while True:
		try:
			async with connect(remoteserver) as websocket:
				await websocket.send("REGISTER/DRONE/TEAM09DRONEPROJECT")
				while True:
					message = await websocket.recv()
					async with globallock:
						lastcontrolled = time.time()
					await handleMessage(message, websocket)
		except Exception as e:
			print(e)
			break
	'''# Assume server is down
	print("Swapping to Hotspot mode")
	subprocess.run(["nmcli", "con", "up", "id", "Hotspot"])
	while True:
		try:
			async with connect(localserver) as websocket:
				await websocket.send("REGISTER/DRONE/TEAM09DRONEPROJECT")
				while True:
					message = await websocket.recv()
					async with globallock:
						lastcontrolled = time.time()
					await handleMessage(message, websocket)
		except Exception as e:
			print(e)
			break'''

async def noconnectionwatchdog():
	global lastcontrolled
	while True:
		await asyncio.sleep(5)
		async with globallock:
			if time.time() - 30 > lastcontrolled:
				# No control detected for 30 seconds
				print("No control detected")

async def MOCK_TOGGLE_STATE():
	global state
	while True:
		await asyncio.sleep(5)
		async with globallock:
			if state == "Running":
				state = "Waiting"
			elif state == "Waiting":
				state = "Running"

async def main():
	if testing:
		asyncio.create_task(MOCK_TOGGLE_STATE())
	asyncio.create_task(websockethandler())
	asyncio.create_task(noconnectionwatchdog())
	await asyncio.gather(*asyncio.all_tasks() - {asyncio.current_task()})

if __name__ == "__main__":
	asyncio.run(main())