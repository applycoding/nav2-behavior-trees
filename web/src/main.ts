// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { ViewerApp } from "./app";

const app = new ViewerApp();
void app.init().catch((err: unknown) => {
  console.error(err);
  const msg = err instanceof Error ? err.message : String(err);
  document.body.insertAdjacentHTML(
    "beforeend",
    `<p style="color:#fca5a5;padding:1rem">Failed to load viewer: ${msg}</p>`,
  );
});
