import type { NextApiRequest, NextApiResponse } from 'next';
import { serialize } from 'cookie';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse
) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  // Clear the auth cookie by setting it with maxAge 0
  const cookie = serialize('auth-token', '', {
    httpOnly: true,
    secure: process.env.NODE_ENV === 'production',
    sameSite: 'lax',
    maxAge: 0, // This will delete the cookie
    path: '/',
  });

  res.setHeader('Set-Cookie', cookie);

  return res.status(200).json({
    success: true,
    message: 'Logged out successfully',
  });
}
