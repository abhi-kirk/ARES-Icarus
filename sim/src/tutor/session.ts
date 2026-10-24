// A one-hour JWT in an HttpOnly cookie. The page never reads it. The browser
// attaches it to /api calls on its own. This is not a login: it only says this
// browser loaded the site recently. Anyone who asks for a session still gets one.

const COOKIE = "icarus_session";
const HOUR = 60 * 60;
const encoder = new TextEncoder();
const decoder = new TextDecoder();

export function sessionCookie(token: string, secure: boolean): string {
  const secureFlag = secure ? "; Secure" : "";
  return `${COOKIE}=${token}; HttpOnly; Path=/api; Max-Age=${HOUR}; SameSite=Lax${secureFlag}`;
}

export function sessionToken(cookieHeader: string | undefined): string {
  if (!cookieHeader) return "";
  for (const part of cookieHeader.split(";")) {
    const trimmed = part.trim();
    const eq = trimmed.indexOf("=");
    if (eq === -1) continue;
    const name = trimmed.slice(0, eq);
    if (name !== COOKIE) continue;
    return trimmed.slice(eq + 1);
  }
  return "";
}

export async function mintSession(secret: string, nowMs: number): Promise<string> {
  const header = base64Url(encoder.encode(JSON.stringify({ alg: "HS256", typ: "JWT" })));
  const exp = Math.floor(nowMs / 1000) + HOUR;
  const payload = base64Url(encoder.encode(JSON.stringify({ exp })));
  const signingInput = `${header}.${payload}`;
  const key = await hmacKey(secret);
  const sig = new Uint8Array(await crypto.subtle.sign("HMAC", key, encoder.encode(signingInput)));
  return `${signingInput}.${base64Url(sig)}`;
}

export async function sessionIsValid(secret: string, token: string, nowMs: number): Promise<boolean> {
  const parts = token.split(".");
  if (parts.length !== 3) return false;
  const headerPart = parts[0];
  const payloadPart = parts[1];
  const sigPart = parts[2];

  let header: { alg?: string };
  let payload: { exp?: number };
  try {
    header = JSON.parse(decoder.decode(base64UrlDecode(headerPart))) as { alg?: string };
    payload = JSON.parse(decoder.decode(base64UrlDecode(payloadPart))) as { exp?: number };
  } catch {
    return false;
  }
  if (header.alg !== "HS256") return false;
  if (typeof payload.exp !== "number") return false;
  if (payload.exp < Math.floor(nowMs / 1000)) return false;

  try {
    const key = await hmacKey(secret);
    const signingInput = `${headerPart}.${payloadPart}`;
    const signature = bytesAsBuffer(base64UrlDecode(sigPart));
    return await crypto.subtle.verify("HMAC", key, signature, encoder.encode(signingInput));
  } catch {
    return false;
  }
}

function bytesAsBuffer(bytes: Uint8Array): ArrayBuffer {
  const copy = new ArrayBuffer(bytes.byteLength);
  new Uint8Array(copy).set(bytes);
  return copy;
}

async function hmacKey(secret: string): Promise<CryptoKey> {
  return crypto.subtle.importKey(
    "raw",
    encoder.encode(secret),
    { name: "HMAC", hash: "SHA-256" },
    false,
    ["sign", "verify"],
  );
}

function base64Url(bytes: Uint8Array): string {
  let binary = "";
  for (let i = 0; i < bytes.length; i++) {
    binary += String.fromCharCode(bytes[i]);
  }
  return btoa(binary).replaceAll("+", "-").replaceAll("/", "_").replaceAll("=", "");
}

function base64UrlDecode(value: string): Uint8Array {
  const normalized = value.replaceAll("-", "+").replaceAll("_", "/");
  const pad = normalized.length % 4 === 0 ? "" : "=".repeat(4 - (normalized.length % 4));
  const binary = atob(normalized + pad);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) {
    bytes[i] = binary.charCodeAt(i);
  }
  return bytes;
}
