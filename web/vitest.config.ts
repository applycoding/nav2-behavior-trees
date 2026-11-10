// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    include: ["test/**/*.test.ts"],
  },
});
