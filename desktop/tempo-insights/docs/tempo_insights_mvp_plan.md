# Tempo Insights MVP Build Plan

Each task is small, testable, and focused on one concern.  

---

## Phase 0 — Project & Environment

- **Task 1: Initialize repo**  
  Do: `pnpm dlx create-next-app@latest` (TS, App Router off, ESLint on)  
  Done when: repo builds and runs `next dev` without errors.

- **Task 2: Add core deps**  
  Do: add `@mantine/core @mantine/hooks @mantine/notifications recharts d3 prisma @prisma/client zod bcrypt jsonwebtoken`  
  Done when: `pnpm install` succeeds and `pnpm build` compiles.

- **Task 3: Project scripts**  
  Do: add scripts `dev`, `build`, `start`, `worker:bt`, `worker:analysis`, `prisma:generate`, `prisma:migrate`  
  Done when: `pnpm run` lists all scripts.

- **Task 4: Repo hygiene**  
  Do: add `.editorconfig`, `.nvmrc`, `.gitignore` (Node + Next + env + prisma)  
  Done when: tracked files are clean on `git status`.

- **Task 5: CI sanity (optional)**  
  Do: GitHub Action that runs `pnpm i`, `pnpm build`, `pnpm prisma generate`  
  Done when: CI green on main.

---

## Phase 1 — Theming & App Shell

- **Task 6: Mantine provider**  
  Wrap app in `<MantineProvider>` with `colorScheme="dark"`  
  Done when: any page renders Mantine styles.

- **Task 7: Theme palette**  
  Implement theme file with colors from spec; apply overrides.  
  Done when: background is `#002233` and text matches palette.

- **Task 8: Notifications provider**  
  Add `<Notifications />` top-level.  
  Done when: demo toast renders.

- **Task 9: App layout**  
  Base layout with left nav, center content, right sidebar.  
  Done when: three columns visible on `/home`.

---

## Phase 2 — Database & Prisma

- **Task 10: Prisma init**  
  Run `npx prisma init`, set `DATABASE_URL`.  
  Done when: `prisma generate` works.

- **Task 11: User & Role models**  
  Add `User`, `UserRole`.  
  Done when: migrate runs; tables exist.

- **Task 12: Device model**  
  Fields: ownerId, lentToId, state, bluetoothId, name, lastSeen.  
  Done when: migrate shows columns.

- **Task 13: JumpLog model**  
  Fields: rawLog, hash, offsets, flags.  
  Done when: migrate succeeds.

- **Task 14: Group & Membership models**  
  `Group`, `GroupMember`.  
  Done when: migrate succeeds.

- **Task 15: FormationSkydive models**  
  `FormationSkydive`, `FormationParticipant`.  
  Done when: migrate succeeds.

- **Task 16: Invitation model**  
  `UserInvitation`.  
  Done when: migrate succeeds.

- **Task 17: Seed admin**  
  Insert admin user.  
  Done when: seeding works.

---

## Phase 3 — Authentication

- **Task 18: Hash utility**  
  Implement bcrypt wrapper.  
  Done when: test passes.

- **Task 19: JWT config**  
  Env secret, issue 30-day JWT.  
  Done when: decode test OK.

- **Task 20: Login API**  
  `/api/auth/login`.  
  Done when: cookie set.

- **Task 21: Register API**  
  `/api/auth/register`.  
  Done when: new user logs in.

- **Task 22: Auth middleware**  
  Verify cookie + load user.  
  Done when: protected API denies anon.

- **Task 23: Role guard**  
  `requireAdmin`.  
  Done when: test blocks non-admin.

- **Task 24: Login page**  
  Mantine form.  
  Done when: redirects `/home`.

- **Task 25: Register page**  
  Auto-login + redirect.  
  Done when: works.

- **Task 26: Logout**  
  Clear cookie API.  
  Done when: `/home` redirects.

---

## Phase 4 — Navigation & Pages Skeletons

- **Task 27: Route guard HOC**  
  Redirects to `/login`.  
  Done when: `/home` requires login.

- **Task 28: /home skeleton**  
  Left nav + placeholders.  
  Done when: visible.

- **Task 29: /profile skeleton**  
  Editable profile form.  
  Done when: page loads.

- **Task 30: /devices skeleton (admin)**  
  List placeholder.  
  Done when: non-admin blocked.

- **Task 31: /users/[slug] skeleton**  
  User card.  
  Done when: seeded admin loads.

- **Task 32: /groups/[slug] skeleton**  
  Group info.  
  Done when: seeded group loads.

- **Task 33: /review/fs/[id] skeleton**  
  Canvas placeholder.  
  Done when: loads.

---

## (Remaining Phases)

The plan continues through:  
- **Phase 5: Users & Groups Basics**  
- **Phase 6: Devices (Admin)**  
- **Phase 7: Jump Logs**  
- **Phase 8: Bluetooth Scanner Worker**  
- **Phase 9: Analysis Worker**  
- **Phase 10: Home Panels & Jump Details**  
- **Phase 11: Formation Review**  
- **Phase 12: Lending & Proxy Users**  
- **Phase 13: Bluetooth Admin Actions**  
- **Phase 14: Analysis Enhancements**  
- **Phase 15: Visibility & Permissions**  
- **Phase 16: Export & Deletion**  
- **Phase 17: Polling & Performance**  
- **Phase 18: Deployment**  

Each phase breaks down into atomic tasks with clear “Done when” checks.  
