// pages/api/lending/proxy.ts
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { generateUniqueSlug } from '../../../lib/utils/slug';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { name } = req.body;
  const creatorId = req.user!.id;

  if (!name || typeof name !== 'string' || name.trim().length === 0) {
    return res.status(400).json({ error: 'Name is required' });
  }

  try {
    // Check if user is a proxy (proxies cannot create other proxies)
    const creator = await prisma.user.findUnique({
      where: { id: creatorId },
    });

    if (!creator) {
      return res.status(404).json({ error: 'User not found' });
    }

    if (creator.isProxy) {
      return res.status(403).json({ error: 'Proxy users cannot create other proxy users' });
    }

    // Generate unique slug
    const slug = await generateUniqueSlug(name.trim(), 'user');

    // Create proxy user with a dummy email and password
    const proxyUser = await prisma.user.create({
      data: {
        name: name.trim(),
        slug,
        email: `proxy-${Date.now()}-${Math.random().toString(36).substring(2, 9)}@tempo.local`,
        password: 'PROXY_NO_LOGIN', // This ensures they can't log in
        isProxy: true,
        proxyCreatorId: creatorId,
      },
      select: {
        id: true,
        name: true,
        slug: true,
        isProxy: true,
        createdAt: true,
      },
    });

    console.log(`[PROXY] Created proxy user ${proxyUser.name} (${proxyUser.id}) by ${creator.name}`);

    return res.status(201).json({
      success: true,
      proxyUser,
    });
  } catch (error) {
    console.error('Create proxy error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});