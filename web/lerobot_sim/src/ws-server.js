// ws-server.js
import { WebSocketServer } from "ws";

const wss = new WebSocketServer({ port: 8765 });

wss.on("connection", (ws) => {
    console.log("Client connected");

    // --- Broadcast helper ---
    function broadcast(data) {
        for (const client of wss.clients) {
            if (client.readyState === 1) { // WebSocket.OPEN
                client.send(JSON.stringify(data));
            }
        }
    }

    ws.on("message", (msg) => {
        try {
            const data = JSON.parse(msg);
            // Assume Python sends e.g. { joint1: 30, joint2: 45 }
            broadcast(data);
        } catch (err) {
            console.error("Invalid message from Python:", err);
        }
    });

});

console.log("WebSocket server running on ws://localhost:8765");