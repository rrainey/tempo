This is a [Next.js](https://nextjs.org) project bootstrapped with [`create-next-app`](https://nextjs.org/docs/pages/api-reference/create-next-app).

## Getting Started

First, run the development server:

```bash
npm run dev
# or
yarn dev
# or
pnpm dev
# or
bun dev
```

Open [http://localhost:3000](http://localhost:3000) with your browser to see the result.

You can start editing the page by modifying `pages/index.tsx`. The page auto-updates as you edit the file.

[API routes](https://nextjs.org/docs/pages/building-your-application/routing/api-routes) can be accessed on [http://localhost:3000/api/hello](http://localhost:3000/api/hello). This endpoint can be edited in `pages/api/hello.ts`.

The `pages/api` directory is mapped to `/api/*`. Files in this directory are treated as [API routes](https://nextjs.org/docs/pages/building-your-application/routing/api-routes) instead of React pages.

This project uses [`next/font`](https://nextjs.org/docs/pages/building-your-application/optimizing/fonts) to automatically optimize and load [Geist](https://vercel.com/font), a new font family for Vercel.

## Learn More

To learn more about Next.js, take a look at the following resources:

- [Next.js Documentation](https://nextjs.org/docs) - learn about Next.js features and API.
- [Learn Next.js](https://nextjs.org/learn-pages-router) - an interactive Next.js tutorial.

You can check out [the Next.js GitHub repository](https://github.com/vercel/next.js) - your feedback and contributions are welcome!

## Deploy on Vercel

The easiest way to deploy your Next.js app is to use the [Vercel Platform](https://vercel.com/new?utm_medium=default-template&filter=next.js&utm_source=create-next-app&utm_campaign=create-next-app-readme) from the creators of Next.js.

Check out our [Next.js deployment documentation](https://nextjs.org/docs/pages/building-your-application/deploying) for more details.


## Deploy on Ubuntu 22

```bash
wget https://raw.githubusercontent.com/your-org/tempo-insights/main/scripts/configure-instance.sh
chmod +x configure-instance.sh
sudo ./configure-instance.sh
```


## What This Accomplishes:

### ✅ Task 149: Networks
- Creates `tempo-network` bridge network
- Allows services to communicate by container name

### ✅ Task 150: Database service reference
- Placeholder for Supabase PostgreSQL
- Services connect via hostname `db` (or direct to Supabase container)

### ✅ Task 151: Web service
- Uses `tempo-web:latest` image
- Exposes port 3000
- Environment from `.env.docker`
- Health check enabled

### ✅ Task 152: Bluetooth scanner service
- Uses `tempo-bt-scanner:latest` image
- **network_mode: host** for Bluetooth access
- **cap_add: NET_ADMIN** for network capabilities

### ✅ Task 153: Bluetooth scanner volumes
- `/run/dbus:/run/dbus:ro` - D-Bus socket
- `smpmgr-extensions/plugins` - Plugin directory

### ✅ Task 154: Analysis worker service
- Uses `tempo-analysis:latest` image
- No ports exposed (background worker)

### ✅ Task 155: Service dependencies
- `depends_on` ensures correct startup order
- Database starts first, then web, then workers

### ✅ Task 156: Health checks
- Web service has HTTP health check
- Workers have heartbeat logging

### ✅ Task 157: Restart policies
- `restart: unless-stopped` - auto-restart on failure
- Bluetooth scanner excluded from failures

## Network Architecture

The system uses **two separate Docker Compose stacks**:

```
supabase-stack/docker-compose.yml (14 services)
  └── kong (API gateway on port 8000)
  └── postgres (database on port 5432)
  └── ... other Supabase services

docker-compose.yml (3 services + 1 proxy)
  ├── tempo-web (port 3000)
  ├── tempo-bt-scanner (host network)
  ├── tempo-analysis (no ports)
  └── db (reference to Supabase)
```

## Usage:

### Start Everything:
```bash
# 1. Start Supabase stack first
cd supabase-stack
docker compose up -d
cd ..

# 2. Start application services
docker compose up -d

# 3. View logs
docker compose logs -f tempo-web
docker compose logs -f tempo-bt-scanner
docker compose logs -f tempo-analysis
```

### Check Status:
```bash
docker compose ps
```

### Stop Everything:
```bash
docker compose down
cd supabase-stack && docker compose down
```

## Important Note About Database Connection:

The `db` service is a **placeholder**. In `.env.docker`, services connect to Supabase PostgreSQL using:

```bash
# Option 1: Direct to Supabase container (if on same Docker network)
DATABASE_URL=postgresql://postgres:PASSWORD@postgres:5432/postgres

# Option 2: Via host (always works)
DATABASE_URL=postgresql://postgres:PASSWORD@host.docker.internal:54322/postgres
```

The `configure-instance.sh` script handles this configuration automatically.

---

