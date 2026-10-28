import { cloudflare } from "@cloudflare/vite-plugin";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";
import { flightRunner } from "./flightPlugin";

export default defineConfig({
  plugins: [flightRunner(), react(), cloudflare({ viteEnvironment: { name: "worker" } })],
  server: {
    host: "127.0.0.1",
    port: 5174,
    strictPort: true,
  },
  environments: {
    worker: { build: { sourcemap: true } },
  },
});
