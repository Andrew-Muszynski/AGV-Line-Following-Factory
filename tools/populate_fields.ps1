# Populate Word field codes (table of contents, page numbers) and save.
#
# python-docx writes the TOC field but cannot compute its entries -- only
# Word can walk the headings and resolve page numbers. Without this step the
# delivered document opens with an empty contents list.
#
#   powershell -File tools/populate_fields.ps1 -Path <file.docx>
param([Parameter(Mandatory=$true)][string]$Path)

$full = (Resolve-Path $Path).Path
$w = New-Object -ComObject Word.Application
$w.Visible = $false
$w.DisplayAlerts = 0
try {
    $d = $w.Documents.Open($full, $false, $false)   # not read-only: we save
    foreach ($t in $d.TablesOfContents) { $t.Update() }
    $d.Fields.Update() | Out-Null
    $d.Repaginate()
    $pages = $d.ComputeStatistics(2)
    $entries = if ($d.TablesOfContents.Count -gt 0) {
        $d.TablesOfContents.Item(1).Range.Paragraphs.Count } else { 0 }
    $d.Save()
    $d.Close(0)
    "{0}: {1} pages, {2} contents entries" -f (Split-Path $full -Leaf), $pages, $entries
} finally {
    $w.Quit()
    [System.Runtime.Interopservices.Marshal]::ReleaseComObject($w) | Out-Null
}
