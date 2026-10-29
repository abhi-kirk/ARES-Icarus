// One flight at a time. A POST carries a case. The stream binary writes one
// sample per line until the landing ends, the caller hangs up, or the next
// case replaces this one. GET /health does not start a flight.

import { spawn } from "node:child_process";
import { createServer } from "node:http";
import { timingSafeEqual } from "node:crypto";
import { caseFromRequest } from "./case.mjs";

const port = Number(process.env.PORT || 8080);
const binary = process.env.ICARUS_STREAM || "/app/IcarusStream";
const token = process.env.FLIGHT_TOKEN || "";
const maxBody = 64_000;
const killAfterMs = 400;
const flightCapMs = 130_000;

let active = null;

function stopActive() {
  const child = active;
  if (!child || child.exitCode !== null) return;
  child.kill("SIGTERM");
  setTimeout(() => {
    if (child.exitCode === null) child.kill("SIGKILL");
  }, killAfterMs);
}

function sendJson(res, status, error) {
  const body = JSON.stringify({ error });
  res.writeHead(status, { "content-type": "application/json", "cache-control": "no-store" });
  res.end(body);
}

function bearerOk(header) {
  if (!token) return false;
  const prefix = "Bearer ";
  if (!header || !header.startsWith(prefix)) return false;
  const got = Buffer.from(header.slice(prefix.length));
  const expect = Buffer.from(token);
  if (got.length !== expect.length) return false;
  return timingSafeEqual(got, expect);
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    const chunks = [];
    let size = 0;
    req.on("data", (chunk) => {
      size += chunk.length;
      if (size > maxBody) {
        reject(new Error("case is too large"));
        req.destroy();
        return;
      }
      chunks.push(chunk);
    });
    req.on("end", () => resolve(Buffer.concat(chunks).toString("utf8")));
    req.on("error", reject);
  });
}

function fly(req, res, flight) {
  stopActive();
  const child = spawn(binary, [], { stdio: ["pipe", "pipe", "pipe"] });
  active = child;
  if (!child.stdin || !child.stdout || !child.stderr) {
    child.kill("SIGTERM");
    sendJson(res, 503, "The flight program did not start.");
    return;
  }

  child.stdin.write(JSON.stringify(flight));
  child.stdin.end();

  let started = false;
  let stderr = "";
  const cap = setTimeout(() => {
    if (child.exitCode === null) child.kill("SIGKILL");
  }, flightCapMs);

  child.stderr.on("data", (chunk) => {
    if (stderr.length < 4000) stderr += chunk.toString("utf8");
  });

  const finish = () => {
    clearTimeout(cap);
    if (active === child) active = null;
    if (!res.writableEnded) res.end();
  };

  req.on("close", () => {
    if (!res.writableEnded && child.exitCode === null) child.kill("SIGTERM");
  });

  child.stdout.on("data", (chunk) => {
    if (!started) {
      started = true;
      res.writeHead(200, {
        "content-type": "application/x-ndjson",
        "cache-control": "no-store",
      });
    }
    res.write(chunk);
  });

  child.on("error", () => {
    clearTimeout(cap);
    if (!started) sendJson(res, 503, "The flight program did not start.");
    else finish();
  });

  child.on("close", (code) => {
    clearTimeout(cap);
    if (active === child) active = null;
    if (!started) {
      const detail = stderr.trim() || "The flight stopped before the first sample.";
      sendJson(res, code === 0 ? 200 : 422, detail);
      return;
    }
    finish();
  });
}

const server = createServer((req, res) => {
  const url = new URL(req.url || "/", "http://localhost");
  if (req.method === "GET" && url.pathname === "/health") {
    const body = JSON.stringify({ ok: true });
    res.writeHead(200, { "content-type": "application/json", "cache-control": "no-store" });
    res.end(body);
    return;
  }
  if (req.method !== "POST" || url.pathname !== "/flight") {
    sendJson(res, 404, "Not found.");
    return;
  }
  if (!token) {
    sendJson(res, 503, "The flight program is not open.");
    return;
  }
  if (!bearerOk(req.headers.authorization)) {
    sendJson(res, 401, "Unauthorized.");
    return;
  }

  readBody(req)
    .then((text) => {
      let body;
      try {
        body = JSON.parse(text);
      } catch {
        sendJson(res, 400, "The case is not JSON.");
        return;
      }
      fly(req, res, caseFromRequest(body));
    })
    .catch(() => {
      if (!res.writableEnded) sendJson(res, 400, "The case could not be read.");
    });
});

server.listen(port, "0.0.0.0");
