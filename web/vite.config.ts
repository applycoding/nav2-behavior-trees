// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { defineConfig } from "vite";

export default defineConfig({
  base: "/nav2-behavior-trees/",
  build: {
    outDir: "dist",
    emptyOutDir: true,
  },
  server: {
    port: 5173,
  },
});
