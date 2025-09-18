// pages/api/users/search.ts
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { q } = req.query;
  const currentUserId = req.user!.id;

  try {
    // Build search conditions
    const whereConditions: any = {
      // Exclude the current user
      NOT: { id: currentUserId },
      // Only non-proxy users (proxy users are created via the lending flow)
      isProxy: false,
    };

    // Add search term if provided
    if (q && typeof q === 'string' && q.trim().length > 0) {
      const searchTerm = q.trim();
      whereConditions.OR = [
        { name: { contains: searchTerm, mode: 'insensitive' } },
        { email: { contains: searchTerm, mode: 'insensitive' } },
      ];
    }

    // Get users matching search
    const users = await prisma.user.findMany({
      where: whereConditions,
      select: {
        id: true,
        name: true,
        email: true,
        slug: true,
      },
      take: 20, // Limit results
      orderBy: {
        name: 'asc',
      },
    });

    return res.status(200).json({
      users,
    });
  } catch (error) {
    console.error('Search users error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});