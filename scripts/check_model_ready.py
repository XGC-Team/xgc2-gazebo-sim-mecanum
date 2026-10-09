#!/usr/bin/env python3
"""Query one explicitly bound native entity; creation completes through its receipt."""
import argparse
import json
import sys
if sys.version_info < (3, 8):
    raise RuntimeError('Native readiness requires an explicitly selected Python >= 3.8 and the formal XRPC wheel')
from urllib.parse import quote
from xgc2_xrpc.runtime import Runtime
from xgc2_scene_runtime.simulation_client import SimulationClient

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model_name')
    parser.add_argument('--service-ref-json', required=True)
    parser.add_argument('--target-id', required=True)
    args = parser.parse_args()
    runtime = Runtime(blocking_workers=1, max_calls=2)
    client = None
    try:
        client = SimulationClient(json.loads(args.service_ref_json), runtime=runtime,
                                  local_target=args.target_id, timeout=2.0)
        entity = client.client.json('/v1/entities/'+quote(args.model_name, safe=''), method='GET', timeout=2.0)
        reference = entity.get('entities', [{}])[0].get('ref', {})
        if reference.get('id') != args.model_name or not reference.get('generation'):
            raise ValueError('Native entity identity/generation is absent')
        return 0
    except Exception as error:
        print('check_model_ready: '+str(error), file=sys.stderr)
        return 1
    finally:
        if client is not None:
            client.close()
        runtime.close()

if __name__ == '__main__':
    sys.exit(main())
