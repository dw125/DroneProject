import WebSocket, {WebSocketServer} from 'ws';
import http from 'http';
import fs from 'fs';

const PERMITTED_FILES = ["/control.html", "/control.js"];
const PASSWORD = "TEAM09DRONEPROJECT";

// Usage function
function print_usage() {
	console.log("Usage:\nnode . <Port Number>");
}

// Read server port from command line
// and check that it's valid
const SERVER_PORT = parseInt(process.argv[2], 10);

if(isNaN(SERVER_PORT)) {
	console.log("Not a valid port.");
	print_usage();
	process.exit(1);
}

// Server handler
function contentCheck(fileName) {
        if(fileName.includes(".png")) return "image/png";
        if(fileName.includes(".js")) return "application/javascript";
        if(fileName.includes(".html")) return "text/html";
        if(fileName.includes(".css")) return "text/css";
        return "text/plain";
}

function serveFile(fileName, res) {
    let s = fs.createReadStream("." + fileName);
    s.on('open', function () {
        console.log("Loaded file:", fileName);
        res.setHeader('Content-Type', contentCheck(fileName));
        res.setHeader('X-Robots-Tag', 'noindex');
        s.pipe(res);
    });
    s.on('error', function () {
        console.log("Unable to load file:", fileName);
        res.setHeader('Content-Type', 'text/plain');
        res.setHeader('X-Robots-Tag', 'noindex');
        res.statusCode = 404;
        res.end('Not found');
    });
}

function requestListener(req, res) {
        if(req.url == "/") {
                req.url = "/control.html";
        }
        console.log("Processing request for file:", req.url);
        if(PERMITTED_FILES.includes(req.url)) {
                serveFile(req.url, res);
        } else {
                console.log("Rejected request for file:", req.url);
                res.setHeader('X-Robots-Tag', 'noindex');
                res.writeHead(403);
                res.end('');
        }
}

console.log("Opening server on port:", SERVER_PORT);
const server = http.createServer(requestListener);
server.listen(SERVER_PORT);
const wss = new WebSocketServer({server: server});
console.log("Server opened successfully");

var droneSocket = null;
var controlSockets = [];

wss.on('connection', function connection(ws) {
	console.log("Inbound connection");
	ws.on("message", function message(data) {
		let raw_data = new TextDecoder().decode(data);
		let message_data = raw_data.split("/");
		console.log(message_data);
		if (droneSocket == ws) {
			controlSockets.forEach((ws) => ws.send(raw_data));
		} else if (controlSockets.includes(ws)) {
			if(message_data[0] == "HEARTBEAT") {
				ws.send("HEARTBEAT/SERVER");
			}
			droneSocket?.send(raw_data);
		} else if(message_data[0] == "REGISTER") {
			if(message_data[2] != PASSWORD) {
				console.log("Dropping invalid connection - bad auth");
				ws.send("Invalid auth");
				ws.close(3000, "Invalid auth");
			}
			if(message_data[1] == "DRONE") {
				if(droneSocket != null) {
					console.log("Dropping invalid connection - drone slot busy");
					ws.send("Drone already connected");
					ws.close(1013, "Server busy");	
				} else {
					console.log("Accepted drone");
					droneSocket = ws;
					ws.on("close", function close() {
						droneSocket = null;
					});
				}
			} else if (message_data[1] == "CLIENT") {
				console.log("Accepted client");
				controlSockets.push(ws);
				ws.on("close", function close() {
					controlSockets.splice(controlSockets.indexOf(ws), 1);
				});
			} else {
				console.log("Dropping invalid connection - bad client type");
				ws.send("Unknown client type");
				ws.close(1003, "Unknown client type");
			}
		} else if (message_data[0] == "HEARTBEAT") {
			//Unregistered client sent premature heartbeat; respond but don't forward
			ws.send("HEARTBEAT/SERVER");
		} else {
			//Client not following protocol
			console.log("Dropping invalid connection - no auth");
			ws.send("Invalid auth");
			ws.close(3000, "Invalid auth");
		}
	});
});
