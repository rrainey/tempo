// pages/api/auth/me.ts
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  // Remove password from response
  const { password, ...userWithoutPassword } = req.user!;

  return res.status(200).json({
    user: userWithoutPassword,
  });
});
