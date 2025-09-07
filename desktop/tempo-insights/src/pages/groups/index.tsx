import { Container, Title, Text, Button, Group as MantineGroup } from '@mantine/core';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';
import { IconPlus } from '@tabler/icons-react';
import Link from 'next/link';

export default function GroupsPage() {
  return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <MantineGroup justify="space-between" mb="xl">
            <Title order={2}>Groups</Title>
            <Button leftSection={<IconPlus size={16} />}>
              Create Group
            </Button>
          </MantineGroup>

          <Text c="dimmed">
            No groups yet. For testing, visit{' '}
            <Link href="/groups/test-group">
              /groups/test-group
            </Link>
          </Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
