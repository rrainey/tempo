// components/formation/FormationReview.tsx
import React, { useState, useEffect } from 'react';
import { Grid, Stack, Container, Title, Alert, Loader, Center } from '@mantine/core';
import { IconAlertCircle } from '@tabler/icons-react';
import { FormationViewer } from './FormationViewer';
import { BaseInfoPanel } from './BaseInfoPanel';
import { JumperListPanel } from './JumperListPanel';
import type { GeodeticCoordinates } from '../../lib/formation/types';

interface FormationReviewProps {
  formationId: string;
}

export const FormationReview: React.FC<FormationReviewProps> = ({ formationId }) => {
  const [formationData, setFormationData] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [currentTime, setCurrentTime] = useState(0);
  const [baseJumperId, setBaseJumperId] = useState<string>('');
  
  // Default DZ center - should come from formation data
  const dzCenter: GeodeticCoordinates = formationData?.dzCenter || {
    lat_deg: formationData?.participants[0]?.timeSeries[0]?.location.lat_deg || 33.6320,
    lon_deg: formationData?.participants[0]?.timeSeries[0]?.location.lon_deg || -117.2510,
    alt_m: formationData?.dzElevation_m || 436.5
  };

  useEffect(() => {
    const fetchFormation = async () => {
      try {
        const response = await fetch(`/api/formations/${formationId}`);
        if (!response.ok) {
          throw new Error('Failed to load formation data');
        }
        const data = await response.json();
        setFormationData(data);
        setBaseJumperId(data.baseJumperId);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Unknown error');
      } finally {
        setLoading(false);
      }
    };

    fetchFormation();
  }, [formationId]);

  const handleBaseChange = async (newBaseId: string) => {
    setBaseJumperId(newBaseId);
    // TODO: Persist to database
    // await fetch(`/api/formations/${formationId}/base`, {
    //   method: 'PATCH',
    //   body: JSON.stringify({ baseJumperId: newBaseId })
    // });
  };

  if (loading) {
    return (
      <Center h={400}>
        <Loader size="xl" />
      </Center>
    );
  }

  if (error || !formationData) {
    return (
      <Alert icon={<IconAlertCircle size={16} />} color="red">
        {error || 'Failed to load formation data'}
      </Alert>
    );
  }

  return (
    <Container size="xl" p="md">
      <Stack gap="lg">
        <Title order={2}>Formation Review</Title>
        
        <Grid>
          <Grid.Col span={{ base: 12, lg: 8 }}>
            <FormationViewer
              formation={formationData}
              dzCenter={dzCenter}
              onBaseChange={handleBaseChange}
              onTimeChange={setCurrentTime}
            />
          </Grid.Col>
          
          <Grid.Col span={{ base: 12, lg: 4 }}>
            <Stack gap="md">
              <BaseInfoPanel
                formation={formationData}
                currentTime={currentTime}
                baseJumperId={baseJumperId}
              />
              
              <JumperListPanel
                formation={formationData}
                currentTime={currentTime}
                baseJumperId={baseJumperId}
                dzCenter={dzCenter}
              />
            </Stack>
          </Grid.Col>
        </Grid>
      </Stack>
    </Container>
  );
};