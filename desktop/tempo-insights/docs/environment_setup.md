Looking at your project setup, here are the next steps to get your Ubuntu 22 test system running:

## 1. Install Node.js and pnpm

```bash
# Install Node.js (check your .nvmrc for the version, or use latest LTS)
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt-get install -y nodejs

# Install nvm and switch to node version 20
nvm use 20

# Install pnpm
npm install -g pnpm
```

## 2. Install Dependencies

```bash
cd tempo-insights  # or wherever you cloned the repo
pnpm install
```

## 3. Set up Supabase (PostgreSQL)

Since you're using Supabase as your database, you have two options:

### Option A: Use Supabase Cloud (Easier)
1. Create a free account at https://supabase.com
2. Create a new project
3. Get your database URL from the project settings

### Option B: Run Supabase Locally
```bash
# Install Docker if not already installed
sudo apt-get update
sudo apt-get install docker.io docker-compose

# Install Supabase CLI
npm install supabase

# Initialize Supabase locally
npx supabase init
npx supabase start

#save the secrets and URLs emitted from 'supabase start` in a private text file

# optional
sudo ufw allow 54323
```

## 4. Configure Environment Variables

Create a `.env` file in your project root:

```bash
# Copy the example env file (if it exists)
cp .env.example .env

# Or create it manually
touch .env
```

Add these variables to `.env.local`:

```env
# Database URL from Supabase
DATABASE_URL="postgresql://postgres:password@localhost:54322/postgres"  # for local Supabase
# Or use your Supabase cloud URL

# JWT Secret for authentication (generate a random string)
JWT_SECRET="your-random-jwt-secret-here"

# Worker tokens for internal API access
WORKER_TOKEN="your-random-worker-token"

# Bluetooth discovery settings
DISCOVERY_WINDOW=300

# Next.js settings
NEXTAUTH_URL="http://localhost:3000"
```

To generate random secrets:
```bash
# Generate JWT secret
openssl rand -base64 32

# Generate worker token
openssl rand -hex 32
```

## 5. Run Database Migrations

```bash
# Generate Prisma client
pnpm prisma generate

# Run migrations to create database tables
pnpm prisma migrate dev

# Seed the database with initial admin user (if you have a seed script)
pnpm prisma db seed
```

## 6. Verify mcumgr Installation

Since you mentioned you've already built mcumgr:

```bash
# Ensure bluetooth is installed
sudo apt install bluetooth pi-bluetooth bluez blueman
sudo usermod -aG docker $USER
sudo usermod -a -G bluetooth $USER

# reboot

# Make sure mcumgr is in your PATH
which mcumgr
mcumgr --version

# If not in PATH, add it:
export PATH=$PATH:/path/to/mcumgr
```

## 7. Start the Development Server

```bash
# Open port 3000 (optional)
sudo ufw allow 3000

# Start the Next.js development server
pnpm dev
```

The app should now be running at http://localhost:3000

## 8. (Optional) Test the Workers

If you want to test the Bluetooth scanner or analysis workers:

```bash
# In separate terminals:
pnpm worker:bt        # Bluetooth scanner
pnpm worker:analysis  # Log analysis worker
```

## Troubleshooting Notes:

1. **Bluetooth Permissions**: The Bluetooth scanner will need permissions to access Bluetooth. You might need to run it with sudo or add your user to the appropriate groups:
   ```bash
   sudo usermod -a -G bluetooth $USER
   # Log out and back in for changes to take effect
   ```

2. **Database Connection**: If you have connection issues, verify your DATABASE_URL is correct and the database is running.

3. **Port Conflicts**: If port 3000 is in use, you can change it:
   ```bash
   sudo ufw allow 3000

   PORT=3001 pnpm dev
   ```
