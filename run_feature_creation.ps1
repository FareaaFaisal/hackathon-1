$ErrorActionPreference = 'Stop'
$featureDescription = Get-Content -Path "gemini_temp_desc.txt" -Raw
./.specify/scripts/powershell/create-new-feature.ps1 -Json -ShortName 'define-module-specs' -Number 1 -FeatureDescription "$featureDescription"
