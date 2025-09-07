import { requireAdmin, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';

export default requireAdmin(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  return res.status(200).json({
    message: 'Admin access granted',
    user: {
      email: req.user!.email,
      role: req.user!.role,
    },
  });
});
