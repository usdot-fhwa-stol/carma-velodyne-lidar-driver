# Instructions on how to trigger Dockerhub.yml github workflow manually for a specific branch:

Run the following in your command line with your github token (you can replace the branch name in "ref"):
```
curl -X POST \
  -H "Authorization: Bearer YOUR_PERSONAL_ACCESS_TOKEN" \
  -H "Accept: application/vnd.github+json" \
  https://api.github.com/repos/usdot-fhwa-stol/carma-velodyne-lidar-driver/actions/workflows/dockerhub.yml/dispatches \
  -d '{
    "ref": "develop"
  }'
```
Source: https://docs.github.com/en/actions/managing-workflow-runs-and-deployments/managing-workflow-runs/manually-running-a-workflow
