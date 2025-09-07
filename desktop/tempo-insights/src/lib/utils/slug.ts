import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

// Reserved slugs that match our app routes
const RESERVED_SLUGS = [
  'api',
  'profile',
  'users',
  'groups',
  'home',
  'review',
  'login',
  'register',
  'devices',
  'logout',
  'admin',
  'settings',
  'formations',
  'jumps',
  'public',
  'private',
];

export function createBaseSlug(name: string): string {
  return name
    .toLowerCase()
    .trim()
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/^-+|-+$/g, '');
}

export async function generateUniqueSlug(
  name: string,
  table: 'user' | 'group',
  excludeId?: string
): Promise<string> {
  const baseSlug = createBaseSlug(name);

  // If it's a reserved slug, always add a suffix
  if (RESERVED_SLUGS.includes(baseSlug)) {
    return findUniqueSlug(baseSlug, table, excludeId, 1);
  }

  // Check if base slug is available
  const exists = await checkSlugExists(baseSlug, table, excludeId);
  if (!exists) {
    return baseSlug;
  }

  // Find unique slug with suffix
  return findUniqueSlug(baseSlug, table, excludeId);
}

async function findUniqueSlug(
  baseSlug: string,
  table: 'user' | 'group',
  excludeId?: string,
  startFrom: number = 1
): Promise<string> {
  let counter = startFrom;
  let candidateSlug = `${baseSlug}-${counter}`;

  while (await checkSlugExists(candidateSlug, table, excludeId)) {
    counter++;
    candidateSlug = `${baseSlug}-${counter}`;
  }

  return candidateSlug;
}

async function checkSlugExists(
  slug: string,
  table: 'user' | 'group',
  excludeId?: string
): Promise<boolean> {
  if (table === 'user') {
    const user = await prisma.user.findFirst({
      where: {
        slug,
        ...(excludeId && { NOT: { id: excludeId } }),
      },
    });
    return !!user;
  } else {
    const group = await prisma.group.findFirst({
      where: {
        slug,
        ...(excludeId && { NOT: { id: excludeId } }),
      },
    });
    return !!group;
  }
}
