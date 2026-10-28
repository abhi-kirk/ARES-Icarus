import { spawn, type ChildProcess } from "node:child_process";
import { existsSync } from "node:fs";
import type { IncomingMessage, ServerResponse } from "node:http";
import path from "node:path";
import { fileURLToPath } from "node:url";
import type { Plugin } from "vite";
import { caseFromRequest } from "./src/flight/case";

const simDir = path.dirname(fileURLToPath(import.meta.url));
const binary = path.resolve(simDir, "../engine/build/IcarusStream");

let active: ChildProcess | null = null;

function stopActive(): void {
  const child = active;
  if (!child || child.exitCode !== null) return;
  child.kill("SIGTERM");
  setTimeout(() => {
    if (child.exitCode === null) child.kill("SIGKILL");
  }, 400);
}

function readBody(req: IncomingMessage): Promise<string> {
  return new Promise((resolve, reject) => {
    const chunks: Buffer[] = [];
    let size = 0;
    req.on("data", (chunk: Buffer) => {
      size += chunk.length;
      if (size > 64_000) {
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

function sendJson(res: ServerResponse, status: number, error: string): void {
  const body = JSON.stringify({ error });
  res.writeHead(status, { "content-type": "application/json", "cache-control": "no-store" });
  res.end(body);
}

// Dev-only. The Worker never starts this binary. A knob posts a case, and the
// laptop flies it, streaming one sample per line until the request is closed.
export function flightRunner(): Plugin {
  return {
    name: "icarus-flight",
    configureServer(server) {
      server.middlewares.use("/api/flight", (req, res, next) => {
        if (req.method !== "POST") {
          next();
          return;
        }
        void handle(req, res);
      });
    },
  };
}

async function handle(req: IncomingMessage, res: ServerResponse): Promise<void> {
  if (!existsSync(binary)) {
    sendJson(res, 503, "The flight program is not built.");
    return;
  }

  let text = "";
  try {
    text = await readBody(req);
  } catch {
    sendJson(res, 400, "The case could not be read.");
    return;
  }

  let body: unknown;
  try {
    body = JSON.parse(text) as unknown;
  } catch {
    sendJson(res, 400, "The case is not JSON.");
    return;
  }

  const flight = caseFromRequest(body);
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
  child.stderr.on("data", (chunk: Buffer) => {
    stderr += chunk.toString("utf8");
  });

  const closed = () => {
    if (active === child) active = null;
    if (!res.writableEnded) res.end();
  };

  req.on("close", () => {
    if (!res.writableEnded && child.exitCode === null) child.kill("SIGTERM");
  });

  child.stdout.on("data", (chunk: Buffer) => {
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
    if (!started) sendJson(res, 503, "The flight program did not start.");
    else closed();
  });

  child.on("close", (code) => {
    if (active === child) active = null;
    if (!started) {
      const detail = stderr.trim() || "The flight stopped before the first sample.";
      sendJson(res, code === 0 ? 200 : 422, detail);
      return;
    }
    closed();
  });
}
