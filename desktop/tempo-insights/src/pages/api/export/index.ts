// pages/api/export/index.ts

import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { createClient } from '@supabase/supabase-js';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import archiver from 'archiver';

const prisma = new PrismaClient();

const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

async function handler(req: AuthenticatedRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!req.user) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  const userId = req.user.id;

  try {
    // Gather all user data
    const [user, jumpLogs, devices, groupMemberships, formationParticipations, connections] = await Promise.all([
      prisma.user.findUnique({
        where: { id: userId },
        select: {
          id: true,
          email: true,
          name: true,
          slug: true,
          role: true,
          nextJumpNumber: true,
          createdAt: true,
          isProxy: true,
          homeDropzone: {
            select: {
              id: true,
              name: true,
              slug: true,
              icaoCode: true,
              latitude: true,
              longitude: true,
              elevation: true,
              timezone: true,
            }
          }
        }
      }),
      prisma.jumpLog.findMany({
        where: { userId },
        orderBy: { createdAt: 'asc' },
        include: {
          device: {
            select: {
              name: true,
              bluetoothId: true,
            }
          }
        }
      }),
      prisma.device.findMany({
        where: { ownerId: userId },
        select: {
          id: true,
          bluetoothId: true,
          name: true,
          state: true,
          createdAt: true,
          lastSeen: true,
        }
      }),
      prisma.groupMember.findMany({
        where: { userId },
        include: {
          group: {
            select: {
              id: true,
              name: true,
              slug: true,
              description: true,
              isPublic: true,
            }
          }
        }
      }),
      prisma.formationParticipant.findMany({
        where: { userId },
        include: {
          formation: {
            select: {
              id: true,
              name: true,
              jumpTime: true,
              aircraft: true,
              altitude: true,
              notes: true,
            }
          },
          jumpLog: {
            select: {
              id: true,
              jumpNumber: true,
              exitTimestampUTC: true,
            }
          }
        }
      }),
      prisma.connection.findMany({
        where: {
          OR: [
            { userId1: userId },
            { userId2: userId }
          ]
        },
        include: {
          user1: {
            select: {
              id: true,
              name: true,
              slug: true,
            }
          },
          user2: {
            select: {
              id: true,
              name: true,
              slug: true,
            }
          }
        }
      })
    ]);

    if (!user) {
      return res.status(404).json({ error: 'User not found' });
    }

    // Get formation participants info (names only, not their logs)
    const formationParticipantsInfo = await Promise.all(
      formationParticipations.map(async (fp) => {
        const otherParticipants = await prisma.formationParticipant.findMany({
          where: {
            formationId: fp.formationId,
            userId: { not: userId }
          },
          include: {
            user: {
              select: {
                name: true,
                slug: true,
              }
            }
          }
        });

        return {
          formation: fp.formation,
          myJumpLog: fp.jumpLog,
          otherJumpers: otherParticipants.map(p => ({
            name: p.user.name,
            slug: p.user.slug,
            position: p.position,
          }))
        };
      })
    );

    // Build export data structure
    const exportData = {
      exportDate: new Date().toISOString(),
      user: {
        ...user,
        createdAt: user.createdAt.toISOString(),
      },
      connections: connections.map(c => {
        const otherUser = c.userId1 === userId ? c.user2 : c.user1;
        return {
          name: otherUser.name,
          slug: otherUser.slug,
          connectedSince: c.createdAt.toISOString(),
        };
      }),
      devices,
      groupMemberships: groupMemberships.map(gm => ({
        role: gm.role,
        joinedAt: gm.joinedAt.toISOString(),
        group: gm.group,
      })),
      formationSkydives: formationParticipantsInfo,
      jumpLogs: jumpLogs.map(log => ({
        id: log.id,
        hash: log.hash,
        jumpNumber: log.jumpNumber,
        device: log.device,
        fileSize: log.fileSize,
        storagePath: log.storagePath,
        createdAt: log.createdAt.toISOString(),
        updatedAt: log.updatedAt.toISOString(),
        visibleToConnections: log.visibleToConnections,
        notes: log.notes,
        // Analysis results
        exitTimestampUTC: log.exitTimestampUTC?.toISOString() || null,
        exitLatitude: log.exitLatitude,
        exitLongitude: log.exitLongitude,
        exitAltitudeFt: log.exitAltitudeFt,
        deployAltitudeFt: log.deployAltitudeFt,
        freefallTimeSec: log.freefallTimeSec,
        avgFallRateMph: log.avgFallRateMph,
        exitOffsetSec: log.exitOffsetSec,
        deploymentOffsetSec: log.deploymentOffsetSec,
        landingOffsetSec: log.landingOffsetSec,
      }))
    };

    // Set headers for streaming ZIP
    res.setHeader('Content-Type', 'application/zip');
    res.setHeader('Content-Disposition', `attachment; filename="tempo-export-${user.slug}-${Date.now()}.zip"`);

    // Create archive
    const archive = archiver('zip', {
      zlib: { level: 9 } // Maximum compression
    });

    // Pipe archive to response
    archive.pipe(res);

    // Add JSON metadata file
    archive.append(JSON.stringify(exportData, null, 2), { name: 'export-data.json' });

    // Add raw log files
    for (const log of jumpLogs) {
      if (log.storagePath) {
        try {
          const { data: fileData, error } = await supabase.storage
            .from('jump-logs')
            .download(log.storagePath);

          if (!error && fileData) {
            const buffer = Buffer.from(await fileData.arrayBuffer());
            const fileName = `logs/jump-${log.jumpNumber || 'unknown'}-${log.hash.substring(0, 8)}.bin`;
            archive.append(buffer, { name: fileName });
          }
        } catch (error) {
          console.error(`Failed to download log ${log.id}:`, error);
          // Continue with other files
        }
      }
    }

    // Finalize the archive
    await archive.finalize();

  } catch (error) {
    console.error('Error creating export:', error);
    if (!res.headersSent) {
      res.status(500).json({ error: 'Internal server error' });
    }
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);