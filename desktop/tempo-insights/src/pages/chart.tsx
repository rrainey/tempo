import { Container, Title, Grid, Paper, Text, Stack, Group, Badge, Button, Card, Table } from '@mantine/core';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { MyJumpsPanel } from '../components/home/MyJumpsPanel';
import { FormationJumpsPanel } from '../components/home/FormationJumpsPanel';
import { JumpDetailsPanel } from '../components/home/JumpDetailsPanel';
import { useEffect, useState } from 'react';
import { notifications } from '@mantine/notifications';
import { IconCheck, IconX, IconParachute } from '@tabler/icons-react';
import { useRouter } from 'next/router';
import { TestAltitudeChart } from './test-altitude-chart';

interface Invitation {
  id: string;
  code: string;
  group: {
    id?: string;
    name?: string;
    slug?: string;
  };
  groupRole: string;
  invitedBy: string;
  expiresAt: string;
  createdAt: string;
}

export default function HomePage() {
  const router = useRouter();
  const [invitations, setInvitations] = useState<Invitation[]>([]);
  const [loadingInvitations, setLoadingInvitations] = useState(true);
  const [processingInvite, setProcessingInvite] = useState<string | null>(null);
  const [selectedJumpId, setSelectedJumpId] = useState<string | null>(null);

  useEffect(() => {
    loadInvitations();
  }, []);

  const loadInvitations = async () => {
    try {
      const response = await fetch('/api/users/invitations');
      if (response.ok) {
        const data = await response.json();
        setInvitations(data.invitations);
      }
    } catch (error) {
      console.error('Error loading invitations:', error);
    } finally {
      setLoadingInvitations(false);
    }
  };

  const handleInvitation = async (code: string, accept: boolean) => {
    setProcessingInvite(code);

    try {
      const response = await fetch(`/api/invitations/${code}/${accept ? 'accept' : 'decline'}`, {
        method: 'POST',
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || `Failed to ${accept ? 'accept' : 'decline'} invitation`);
      }

      notifications.show({
        title: 'Success',
        message: data.message,
        color: 'green',
      });

      setInvitations(prev => prev.filter(inv => inv.code !== code));

      if (accept && data.group?.slug) {
        router.push(`/groups/${data.group.slug}`);
      }
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'An error occurred',
        color: 'red',
      });
    } finally {
      setProcessingInvite(null);
    }
  };

  const handleJumpSelect = (jumpId: string) => {
    setSelectedJumpId(jumpId);
    // TODO: Task 77 - Update center panel with jump details
  };

  const handleFormationSelect = (formationId: string) => {
    router.push(`/review/fs/${formationId}`);
  };

  return (
    <TestAltitudeChart/>
  );
}