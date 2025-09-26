// Save as: src/pages/api/users/index.ts

import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const querySchema = z.object({
  limit: z.string().transform(Number).pipe(z.number().min(1).max(100)).optional().default(50),
  search: z.string().optional(),
  excludeProxies: z.string().transform(val => val === 'true').optional().default(false)
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Only admins can list all users
    if (req.user!.role !== 'ADMIN' && req.user!.role !== 'SUPER_ADMIN') {
      return res.status(403).json({ error: 'Only administrators can list users' });
    }

    const query = querySchema.parse(req.query);

    // Build where clause
    const where: any = {};
    
    if (query.excludeProxies) {
      where.isProxy = false;
    }
    
    if (query.search) {
      where.OR = [
        { name: { contains: query.search, mode: 'insensitive' } },
        { email: { contains: query.search, mode: 'insensitive' } }
      ];
    }

    // Get users
    const users = await prisma.user.findMany({
      where,
      select: {
        id: true,
        name: true,
        email: true,
        slug: true,
        role: true,
        nextJumpNumber: true,
        isProxy: true,
        createdAt: true,
        _count: {
          select: {
            jumpLogs: true
          }
        }
      },
      orderBy: {
        name: 'asc'
      },
      take: query.limit
    });

    return res.status(200).json({
      users: users.map(user => ({
        id: user.id,
        name: user.name,
        email: user.email,
        slug: user.slug,
        role: user.role,
        nextJumpNumber: user.nextJumpNumber,
        isProxy: user.isProxy,
        jumpCount: user._count.jumpLogs,
        createdAt: user.createdAt
      }))
    });

  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({ error: 'Invalid query parameters', details: error });
    }

    console.error('List users error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});