import { NextApiRequest, NextApiResponse } from 'next';
import { parse } from 'cookie';
import { verifyJWT } from './jwt';
import { PrismaClient } from '@prisma/client';
import { User, UserRole } from '@prisma/client';

const prisma = new PrismaClient();

export interface AuthenticatedRequest extends NextApiRequest {
  user?: User;
}

export type AuthenticatedApiHandler = (
  req: AuthenticatedRequest,
  res: NextApiResponse
) => void | Promise<void>;

export function withAuth(handler: AuthenticatedApiHandler) {
  return async (req: AuthenticatedRequest, res: NextApiResponse) => {
    try {
      // Parse cookies
      const cookies = parse(req.headers.cookie || '');
      const token = cookies['auth-token'];

      if (!token) {
        return res.status(401).json({ error: 'Unauthorized' });
      }

      // Verify JWT
      const payload = verifyJWT(token);

      // Load user from database
      const user = await prisma.user.findUnique({
        where: { id: payload.userId },
      });

      if (!user) {
        return res.status(401).json({ error: 'Unauthorized' });
      }

      // Attach user to request
      req.user = user;

      // Call the handler
      return handler(req, res);
    } catch (error) {
      console.error('Auth middleware error:', error);
      return res.status(401).json({ error: 'Unauthorized' });
    }
  };
}

export function requireAdmin(handler: AuthenticatedApiHandler) {
  return withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
    // Check if user has admin role
    if (req.user?.role !== UserRole.ADMIN && req.user?.role !== UserRole.SUPER_ADMIN) {
      return res.status(403).json({ error: 'Forbidden: Admin access required' });
    }

    return handler(req, res);
  });
}
