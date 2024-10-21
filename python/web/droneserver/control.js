const host = "ws://" + location.host;
const reconnect_time = 5000;
const heartbeat_timeout = 5;

let ws = null;
let lastdrone = 0;
let lastserver = 0;
let laststatus = "Unknown";

function sendStop() {
	if(ws?.readyState == WebSocket.OPEN) {
		ws.send("STOP");
	}
}

function sendLaunch() {
	if(ws?.readyState == WebSocket.OPEN) {
		ws.send("LAUNCH/" + document.getElementById("height").value + "/" + document.getElementById("time").value);
	}
}

function checkHeartbeat() {
	if(ws?.readyState == WebSocket.OPEN) {
		ws.send("HEARTBEAT/CLIENT");
	}
	if((Date.now() / 1000) > lastserver + heartbeat_timeout) {
		document.getElementById("scon").innerHTML = "Disconnected";
		document.getElementById("scon").style = "color:red";
		document.getElementById("dcon").innerHTML = "Unknown";
		document.getElementById("dcon").style = "color:grey";
		document.getElementById("dstat").innerHTML = "Unknown";
		document.getElementById("dstat").style = "color:grey";
		document.getElementById("height").disabled = true;
		document.getElementById("time").disabled = true;
		document.getElementById("Launch").disabled = true;
		document.getElementById("Launch").style = "background-color:#888888;height:50px;width:200px";
	} else if ((Date.now() / 1000) > lastdrone + heartbeat_timeout) {
		document.getElementById("scon").innerHTML = "Connected";
		document.getElementById("scon").style = "color:lime";
		document.getElementById("dcon").innerHTML = "Disconnected";
		document.getElementById("dcon").style = "color:red";
		document.getElementById("dstat").innerHTML = "Unknown";
		document.getElementById("dstat").style = "color:grey";
		document.getElementById("height").disabled = true;
		document.getElementById("time").disabled = true;
		document.getElementById("Launch").disabled = true;
		document.getElementById("Launch").style = "background-color:#888888;height:50px;width:200px";
	} else {
		document.getElementById("scon").innerHTML = "Connected";
		document.getElementById("scon").style = "color:lime";
		document.getElementById("dcon").innerHTML = "Connected";
		document.getElementById("dcon").style = "color:lime";
		document.getElementById("dstat").innerHTML = laststatus;
		if(laststatus == "Stopped"){
			document.getElementById("dstat").style = "color:red";
			document.getElementById("height").disabled = false;
			document.getElementById("time").disabled = false;
			document.getElementById("Launch").disabled = false;
			document.getElementById("Launch").style = "background-color:#00FF00;height:50px;width:200px";
		} else if (laststatus == "Running"){
			document.getElementById("dstat").style = "color:lime";
			document.getElementById("height").disabled = true;
			document.getElementById("time").disabled = true;
			document.getElementById("Launch").disabled = true;
			document.getElementById("Launch").style = "background-color:#888888;height:50px;width:200px";
		}  else if (laststatus == "Waiting"){
			document.getElementById("dstat").style = "color:orange";
			document.getElementById("height").disabled = true;
			document.getElementById("time").disabled = true;
			document.getElementById("Launch").disabled = true;
			document.getElementById("Launch").style = "background-color:#888888;height:50px;width:200px";
		} else {
			document.getElementById("dstat").innerHTML = "Unknown";
			document.getElementById("dstat").style = "color:grey";
			document.getElementById("height").disabled = false;
			document.getElementById("time").disabled = false;
			document.getElementById("Launch").disabled = false;
			document.getElementById("Launch").style = "background-color:#00FF00;height:50px;width:200px";
		}
	}
	setTimeout(checkHeartbeat, 1000);
}

function loaded() {
	setTimeout(checkHeartbeat, 1000);
	initConnection();
}

function initConnection() {
	ws = new WebSocket(host);
	ws.addEventListener("open", (event) => {
		ws.send("REGISTER/CLIENT/TEAM09DRONEPROJECT");
	});
	ws.addEventListener("message", (event) => {
		m_args = event.data.split("/");
		console.log("Received message", m_args);
		if(m_args[0] == "HEARTBEAT") {
			if(m_args[1] == "SERVER") {
				lastserver = (Date.now() / 1000);
			}
			if(m_args[1] == "DRONE") {
				lastdrone = (Date.now() / 1000);
				laststatus = m_args[2];
			}
		}
	});
	ws.addEventListener("close", (event) => {
		setTimeout(initConnection, reconnect_time);
	});
	ws.addEventListener("error", (event) => {
		ws.close();
	});
};
