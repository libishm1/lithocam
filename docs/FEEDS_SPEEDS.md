# Feeds and speeds reading for lithocam

Date: 2026-11-12
Scope: light, read-only web reading. Stone and wood cutting data, plus robot notes.
Status: draft reading notes. Not validated. Not promoted to wiki.

## Reading rules used

- Every number below has a source URL. No number was invented.
- Source type: `vendor` (tool maker or dealer page), `vendor blog` (tool seller article, basis not stated), `paper` (peer reviewed or conference), `trade press`, `web table` (reference site, not a tool maker).
- Confidence: `page read` means WebFetch returned the page and the value was read there. `snippet` means the value came from a search-result excerpt only. Snippet values need a page check before use.
- `(conv.)` marks a unit conversion I made. `(inf.)` marks an inference I made. Neither is in the source.
- Conversions: 1 in = 25.4 mm. 1 US gal = 3.785 L. Surface speed v [m/s] = pi x D [mm] x n [rpm] / 60000.

## Table S: stone

### S.1 Segmented disc saws (bridge saw, block cutter, lab blade)

| Material | Tool kind | Tool size | Quantity | Value | Unit | Source | Type | Confidence | Note |
|---|---|---|---|---|---|---|---|---|---|
| Hard granite | Segmented disc saw | not stated | Peripheral speed | 25-30 | m/s | https://sangtools.com/blog/regular-stone-material-saw-blade-cutting-speed_b12 | vendor | page read | Column is labelled "feeding speed (m/s)". Values fit peripheral speed (inf.) |
| Mid-hard granite | Segmented disc saw | not stated | Peripheral speed | 30-35 | m/s | same | vendor | page read | same label issue |
| Soft granite | Segmented disc saw | not stated | Peripheral speed | 35-40 | m/s | same | vendor | page read | same label issue |
| Hard marble | Segmented disc saw | not stated | Peripheral speed | 40-45 | m/s | same | vendor | page read | same label issue |
| Soft marble | Segmented disc saw | not stated | Peripheral speed | 45-50 | m/s | same | vendor | page read | same label issue |
| Sandstone | Segmented disc saw | not stated | Peripheral speed | 50-65 | m/s | same | vendor | page read | same label issue |
| Granite | Disc saw | not stated | Feed at 6 mm depth | 200-500 | mm/min | same | vendor | page read | page: "cutting depth is 6mm" |
| Marble | Disc saw | not stated | Feed at 6 mm depth | 1000-2000 | mm/min | same | vendor | page read | |
| Quartz / ceramic | Disc saw | not stated | Feed at 6 mm depth | 250-400 | mm/min | same | vendor | page read | |
| Granite | Bridge saw blade | 14-18 in examples | Peripheral speed | 45-60 | m/s | https://dynamicstonetools.com/blogs/news/rpm-feed-rate-water-flow-what-the-blade-selector-cant-tell-you | vendor blog | page read | Conflicts with Sang 25-40 m/s |
| Granite | Bridge saw blade | n/a | Water flow | 6-8 GPM = 22.7-30.3 | L/min (conv.) | same | vendor blog | page read | |
| Marble / limestone | Bridge saw blade | n/a | Water flow | 4-6 GPM = 15.1-22.7 | L/min (conv.) | same | vendor blog | page read | |
| Quartzite | Bridge saw blade | n/a | Water flow | 7-9 GPM = 26.5-34.1 | L/min (conv.) | same | vendor blog | page read | |
| Engineered quartz | Bridge saw blade | n/a | Water flow | 6-7 GPM = 22.7-26.5 | L/min (conv.) | same | vendor blog | page read | |
| Granite | Bridge saw blade | n/a | Feed | 1-2 in/min = 25-51 | mm/min (conv.) | same | vendor blog | page read | Far below Sang 200-500 mm/min. Treat as suspect (inf.) |
| Granite (one rock) | Circular diamond blade, lab | not stated | Optimum for min. specific energy | Vp 25 m/s; traverse 70 cm/min; depth 20 mm; coolant 100 mL/s (= 6 L/min conv.) | mixed | https://link.springer.com/article/10.1007/s11771-014-1949-9 (Karakurt 2014, J. Cent. South Univ.) | paper | snippet | Optimum for energy, not tool life |
| Granite (two rocks) | Circular blade, lab | 400 mm | Peripheral speed held constant | 30 | m/s | https://dergipark.org.tr/tr/download/article-file/320532 | paper | snippet | Sawing rates 100, 150, 200 cm2/min |
| 16 rock types | Circular blade | not stated | Vp; feed; depth | 65 m/s; 0.2-1.0 m/min; 10-90 mm | mixed | https://peeref.com/works/80299932 | paper | snippet | Source and units not checked |
| Granite | Small blades, tile lines | not stated | Peripheral speed; feed; depth | up to 35 m/s; 9-12 m/min; 1-10 mm (large blades on blocks 1-2 mm) | mixed | https://diamondtoolmanufacturer.blogspot.com/2013/08/the-quality-diamond-saw-blades.html | vendor blog | snippet | |
| Rock, general | Diamond rock saw | not stated | Tip speed | 40-50 | m/s | https://echidna.com.au/KnowledgeCorner/Articles.php?Art=4 | vendor | snippet | |

### S.2 Diamond wire

| Material | Tool kind | Tool size | Quantity | Value | Unit | Source | Type | Confidence | Note |
|---|---|---|---|---|---|---|---|---|---|
| Granite | Diamond wire, single-wire machine | not stated | Cutting rate | 0.9-1.5 | m2/h | https://link.springer.com/article/10.1007/s00170-021-07577-3 (Konstanty 2021, IJAMT) | paper | snippet | Abstract level |
| Granite | Diamond wire, multi-wire (MDW) | not stated | Cutting rate | 0.6-1.0 | m2/h | same | paper | snippet | |
| Granite | Diamond wire | not stated | Wire speed, typical | 24-30 | m/s | same | paper | snippet | |
| Granite | Diamond wire | 7.4 mm | Worked example | 28.8 m/s at 0.66 m2/h | mixed | same | paper | snippet | |
| Granite | Diamond wire | not stated | Tension, preset | 2000-2400 | N | same | paper | snippet | Rise during cut about 3.5 % of preset |
| Granite | Quarry wire (South Africa) | not stated | Average cutting rate | 3.5 | m2/h | https://www.stonebusiness.us/?p=622 (Jennings 2004, Stone Business) | trade press | page read | Older field figure |
| Granite | Multi-wire, 50 wires | not stated | Wire speed; feed | 33 m/s; 20 cm/h | mixed | same | trade press | page read | 2 cm slabs, 130 kW |
| Marble | Diamond wire | not stated | Line speed; cutting rate | soft: 25-35 m/s, 8-12 m2/h; mid-hard: 22-32 m/s, 6-10 m2/h | mixed | one of https://xmbestlink.en.made-in-china.com/product/oFGEfeCwXAkT/China-Diamond-Wires-Saw-for-Granite-Marble-Block-Cutting.html or https://m.made-in-china.com/product/D12-5-Wire-Rope-for-Poland-Market-1932766115.html | vendor | snippet | Snippet did not say which listing. Check before use |
| Any stone | Pellegrini wire machines | n/a | Wire speed range; feed range | 0-40 m/s; 0-999 cm/h | mixed | https://pdf.directindustry.com/pdf/pellegrini/robot-wire-catalogue/100449-281065.html | vendor | snippet | Machine capability, not a setting |

### S.3 CNC finger bits, profile tools, electroplated tools, milling

| Material | Tool kind | Tool size | Quantity | Value | Unit | Source | Type | Confidence | Note |
|---|---|---|---|---|---|---|---|---|---|
| Granite, marble, quartz | Sintered finger bit, Weha Top Racer, 8 seg | 20 or 22 x 40 mm (page inconsistent) | rpm; feed | 5500-8500 rpm; 24-36 in/min = 610-914 mm/min (conv.) | mixed | https://easternmarble.com/shop/stone-fabrication-supplies/finger-bits/weha-top-racer-high-speed-cnc-diamond-finger-bit/ | vendor (dealer) | page read | 20 mm at 5500-8500 rpm = 5.8-8.9 m/s (inf.) |
| Granite, marble, quartz | Finger bit, Weha Top Sprinter, 8 seg | 20 x 40 mm | rpm; feed | 5000-5800 rpm at one seller, 5000-8500 at another; 12-20 in/min = 305-508 mm/min (conv.) | mixed | https://diamondtoolstore.com/products/top-sprinter-cnc-finger-bit and https://www.usagranitetools.com/?p=237130 | vendor (dealer) | snippet | Snippet did not say which seller gave which rpm |
| Granite, marble, quartz | Finger bit, Puma green, 5 seg | 20 x 40 mm | rpm; feed | 4500-5200 rpm; 10-16 in/min = 254-406 mm/min (conv.) | mixed | https://www.diamondtoolstore.com/puma-cnc-finger-bit-20mm-x-40mm-5-segment-green/ | vendor (dealer) | snippet | Page returned 403 |
| Granite, marble, quartz | Finger bit, Weha Matrix, 7 seg | 22 x 40 mm | rpm; feed | 5500 rpm; 18-24 in/min = 457-610 mm/min (conv.) | mixed | https://www.gmrqualitystoneproducts.com/?p=27306 | vendor (dealer) | snippet | |
| Soft marble / travertine | CNC router bit, edge profiling | not stated | CNC spindle rpm | 8000-12000 | rpm | https://dynamicstonetools.com/blogs/news/router-bit-speeds-and-feed-rates-for-stone-edge-profiling | vendor blog | page read | No diameter. Conflicts with finger-bit rpm above |
| Standard granite | same | not stated | CNC spindle rpm | 10000-14000 | rpm | same | vendor blog | page read | |
| Hard granite / Absolute Black | same | not stated | CNC spindle rpm | 12000-16000 | rpm | same | vendor blog | page read | Nearest entry to charnockite (inf.) |
| Quartzite | same | not stated | CNC spindle rpm | 12000-16000 | rpm | same | vendor blog | page read | |
| Engineered quartz | same | not stated | CNC spindle rpm | 10000-14000 | rpm | same | vendor blog | page read | |
| Limestone / sandstone | same | not stated | CNC spindle rpm | 7000-10000 | rpm | same | vendor blog | page read | |
| Natural stone | CNC edge profile | n/a | Feed by profile | eased 800-1200; bullnose 500-900; ogee 400-700; cove 450-750; miter 300-600; bevel 700-1100; dupont 500-850 | mm/min | same | vendor blog | page read | Not split by stone |
| Natural stone | CNC edge profile | n/a | Depth per pass | miter 2-3 mm; profiles >15 mm: 2-4 roughing passes at 50-70 % depth, then full-depth finish | mm | same | vendor blog | page read | |
| Natural stone | CNC router bit | n/a | Water flow to contact point | 3-5 | L/min | same | vendor blog | page read | Contact below 60 C |
| Granite, engineered stone | Profile wheel, Stonegate F30 | 120 mm | rpm; feed | 5500-6500 rpm; segmented 400-5000, continuous 1500-7000 mm/min | mixed | https://stonegatetooling.com/f-ogee-f30-r17-stone-cnc-profile-wheels.html | vendor | snippet | 403 on fetch. 120 mm at 5500-6500 rpm = 34.6-40.8 m/s (inf.) |
| Marble | Electroplated rebate wheel | 50 mm | rpm; feed | 4500-5500 rpm; 1000-2000 mm/min | mixed | https://amastone.com/?p=155003 | vendor | snippet | |
| Marble, ceramic, granite | Electroplated engraving bit | not stated | rpm; feed; depth per pass | 7000-8000 rpm; 100-500 mm/min; max 4 mm, 2 mm recommended | mixed | https://amastone.com/shop/90-electroplated-engraving-router-bit-diamar/ | vendor | snippet | 403 on fetch |
| Marble | Vacuum-brazed diamond-coated milling tool (lab) | not stated | rpm; feed; depth; water | 6000-12000 rpm; 1500-3000 mm/min; 1 mm; 3 L/min | mixed | https://sage.cnpereading.com/doi/10.1177/17515831261437884 | paper | snippet | Test ranges, not recommendations. WAF blocked fetch |
| Marble | Diamond milling (prior work cited in above) | not stated | Reported optimum | 8000 rpm; 90.65 mm/min; radial 0.10 mm; axial 1.37 mm | mixed | same | paper (secondary) | snippet | |
| Seven natural stones | End mill | 4.0 mm | Test levels | depth 0.1 and 2.0 mm; feed 2000 and 4000 mm/min | mixed | https://www.scinapse.io/papers/3195869294 (Arab. J. Geosci. 2021) | paper | snippet | Test levels only |
| Postaer sandstone vs Carrara marble | End mill | not stated | Finding | no significant parameter effect in sandstone; depth of cut matters in marble | qualitative | https://icscm.ub.rub.de/index.php/ICSCM/article/download/387/344/1893 | paper | snippet | Machine tool, not robot |

### S.4 Core bits, cup wheels, calibrating, polishing pads

| Material | Tool kind | Tool size | Quantity | Value | Unit | Source | Type | Confidence | Note |
|---|---|---|---|---|---|---|---|---|---|
| Not stone specific | Diamond core bit | 2 / 3 / 4 / 5 / 6 / 8 / 10 / 14 / 18 in | rpm min-ideal-max | 2": 1200-1600-2000; 3": 800-1050-1300; 4": 600-800-1000; 5": 475-640-800; 6": 400-560-665; 8": 300-400-500; 10": 240-320-400; 14": 170-225-285; 18": 130-175-220 | rpm | https://www.mkdiamond.com/coring/tec_speed.html | vendor | page read | Ideal is about 4.3 m/s at every size (inf.). 12" max printed as "30" (typo) |
| Granite, marble, quartz | CNC core bit, RockPecker | not stated | rpm; feed | 1400-2800 rpm; 3-4 in/min = 76-102 mm/min (conv.) | mixed | https://www.diamondtoolstore.com/rockpecker-cnc-core-bit/ | vendor (dealer) | snippet | 403 on fetch |
| Quartzite, granite, marble | Thin-wall CNC core bit, Apexx | 1-3/8 in (35 mm conv.) | rpm | 1500-2000, wet only | rpm | https://dynamicstonetools.odoo.com/shop/23801-01007-apexx-green-cnc-thin-wall-core-bit-6mm-14-x-4-x-12-gas-69447 | vendor (dealer) | snippet | |
| Stone, concrete | Diamond cup wheel, max rating | 125 mm; 150 mm (PCD) | Max rpm | 12200; 10200 | rpm | https://metabo.com/au/en/accessories/cutting-grinding/diamond-cup-grinding-wheels | vendor | snippet | About 80 m/s (inf.). Maximum, not a working value |
| Stone | Ring cup wheel | 100 mm | rpm | 11000 | rpm | https://stonegatetooling.com/product/ring-cup-grinding-wheel/ | vendor | snippet | Site calls it recommended. About 58 m/s (inf.) |
| Marble, slate, granite | Resin cup wheel (not diamond), Tyrolit | 110 mm | Max circumferential speed | 50 | m/s | https://www.hoffmann-group.com/US/en/hus/p/895679- | vendor (dealer) | snippet | |
| Slabs | Calibrating machine | n/a | Belt speed | 0.5-4 | m/min | https://dynamicstonetools.com/blogs/news/thickness-calibration-machines-for-stone-a-shop-guide | vendor blog | snippet | No head peripheral speed found |
| Granite | Alpha Ceramica resin wet pad | 4 in | rpm | optimal 2000-2500; max 4000 | rpm | https://dynamicstonetools.com/products/alpha-ceramica-resin-wet-polishing-pad-for-granite | vendor (dealer) | snippet | Snippet said "one retailer". Page attribution not confirmed |
| Granite | Alpha Ceramica EX wet pad | 4 in | Max rpm | 3000 | rpm | https://dynamicstonetools.odoo.com/shop/ex40200r-200-wet-polishing-pad-ceramica-ex-alpha-r-14609/document/576 | vendor | snippet | |
| Stone | Weha Triad 3-step pad | 4 in | Max rpm | 5000 | rpm | https://www.usagranitetools.com/?p=238133 | vendor (dealer) | snippet | Maximum, not working value |
| Stone | Weha Xubi honeycomb pad | 4 in | Max rpm | 5000 | rpm | https://www.diamondtoolstore.com/xubi-dry-honeycomb-diamond-polishing-pads/ | vendor (dealer) | snippet | Maximum, not working value |
| Stone | Weha 3-step hybrid pad | 4 in | Max rpm | 4500 | rpm | https://gmrqualitystoneproducts.com/?p=27084 | vendor (dealer) | snippet | Maximum, not working value |
| Granite | Resin pads, wet, by grit | 4 in | rpm by grit | 50-100: 1500-2000; 200-400: 2000-2500; 800-1500: 2500-3000; 3000 + buff: 3000-3500 | rpm | https://slabwise.com/guides/what-diamond-polishing-pads-are-best-for-granite-finishing | web guide, basis not stated | snippet | Only grit-by-rpm table found |

### S.5 Waterjet

| Material | Tool kind | Tool size | Quantity | Value | Unit | Source | Type | Confidence | Note |
|---|---|---|---|---|---|---|---|---|---|
| Marble 12.7 mm (0.50 in) | Abrasive waterjet, 60000 psi, 0.012 in orifice | n/a | Traverse | 13-20 ipm = 330-508 | mm/min (conv.) | https://www.kmtwaterjet.com/KMT%20Orifice-Cutting%20Speeds%20Data.pdf | vendor | snippet | PDF not opened |
| Marble 19 mm (0.75 in) | same | n/a | Traverse | 6-9 ipm = 152-229 | mm/min (conv.) | same | vendor | snippet | |
| Granite 30 mm, full quality | Abrasive waterjet | n/a | Traverse | 3-8 ipm = 76-203 | mm/min (conv.) | https://dynamicstonetools.com/blogs/news/waterjet-cutting-for-stone-how-it-works-when-to-use-it | vendor blog | page read | Pressure 40000-90000 psi. Abrasive 0.5-1.5 lb/min |

## Table W: wood and wood composites

Feed formula, stated by every vendor source read: feed = rpm x flutes x chip load. Example on the Rennie page: 0.32 mm x 18000 rpm x 2 = 11520 mm/min for a 6 mm 2-flute up-cut bit in MDF.

### W.1 Chip load per tooth, solid carbide router bits (Rennie Tool, mm/tooth)

Source: https://www.rennietool.co.uk/blogs/news/solid-carbide-router-bits-speeds-and-feeds . Type: vendor. Confidence: page read. Basis stated on page: 18000 rpm for most 1, 2 and 3 flute up-cut or down-cut bits; depth of cut rule of thumb 1 x D. Compression bits not covered. No softwood column.

| Material | 2 mm | 3.175 mm | 4 mm | 6 mm | 6.35 mm | 8 mm | 9.35 mm | 10 mm | 12 mm | 12.7 mm |
|---|---|---|---|---|---|---|---|---|---|---|
| Hardwood | .038-.064 | .076-.127 | .102-.153 | .220-.270 | .229-.279 | .310-.370 | .381-.457 | .400-.477 | .460-.513 | .483-.534 |
| Soft plywood | .056-.077 | .102-.153 | .120-.180 | .270-.320 | .279-.330 | .360-.420 | .432-.508 | .445-.520 | .500-.560 | .533-.584 |
| MDF / particle board | .047-.082 | .102-.178 | .140-.210 | .320-.396 | .330-.406 | .430-.500 | .508-.584 | .520-.590 | .613-.660 | .635-.686 |
| High pressure laminate | .039-.065 | .076-.127 | .102-.163 | .219-.294 | .229-.305 | .311-.381 | .381-.457 | .410-.490 | .546-.595 | .584-.635 |

The inch columns match the idcwoodcraft inch table in W.2 after conversion (for example hardwood 6.35 mm .229-.279 mm = .009-.011 in). Both likely share one origin chart (inf.).

### W.2 Other chip-load and feed rows

| Material | Tool kind | Diameter | Flutes | rpm | Chip load | Feed / other | Source | Type | Confidence |
|---|---|---|---|---|---|---|---|---|---|
| Hardwood / softwood / MDF | Router bit, generic | 1/8 in | not stated | not stated | .003-.005 / .004-.006 / .004-.007 in | | https://idcwoodcraft.com/pages/chipload-calculator | vendor (bit seller) | snippet |
| Hardwood / softwood / MDF | same | 1/4 in | not stated | not stated | .009-.011 / .011-.013 / .013-.016 in | | same | vendor | snippet |
| Hardwood / softwood / MDF | same | 3/8 in | not stated | not stated | .015-.018 / .018-.020 / .020-.023 in | | same | vendor | snippet |
| Hardwood / softwood / MDF | same | 1/2 in | not stated | not stated | .019-.021 / .021-.023 / .025-.027 in | | same | vendor | snippet |
| Hardwood | 60 deg V cutter | 1 in | 2 | 18000 | .004-.006 in | 1 x D | https://engineersedge.com/manufacturing/hardwood_speeds_and_feeds_table_15169.htm | web table | page read |
| Hardwood | Straight V end mill | 1/4 in | 1 | 18000 | .005-.007 in | 90-126 ipm; 1 x D | same | web table | page read |
| Hardwood | Straight V end mill | 1/2 in | 2 | 18000 | .006-.008 in | 1 x D | same | web table | page read |
| Hardwood | Upcut end mill | 1/4 in | 2 | 18000 | .006-.008 in | 1 x D | same | web table | page read |
| Hardwood | Downcut end mill | 1/4 in | 2 | 18000 | .005-.007 in | 1 x D | same | web table | page read |
| Hardwood | Upcut end mill | 1/4 in | 1 | 18000 | .004-.006 in | 1 x D | same | web table | page read |
| Hardwood | Tapered upcut ball end mill | 1/8 in | 2 | 18000 | .003-.005 in | 1 x D | same | web table | page read |
| Hardwood | Surfacing cutter | 1-1/4 in | 2 | 18000 | not given | 200-600 ipm; 1/2-3/4 x D; max cut 1/8 in | same | web table | page read |
| MDF | Same rows as hardwood | | | | identical values to hardwood page | surfacing cutter 12000-16000 rpm | https://www.engineersedge.com/manufacturing/medium_density_fiberboard_15170.htm | web table | page read |
| Laminated plywood | 60 deg V / 1/4 V (1 fl) / 1/2 V (2 fl) | 1 / 1/4 / 1/2 in | 2 / 1 / 2 | 18000 | .004-.006 / .006-.008 / .008-.010 in | 1 x D | https://engineersedge.com/manufacturing/laminated_plywood_cnc_speeds_and_feeds_table_15173.htm | web table | page read |
| Laminated plywood | Surfacing cutter | 1-1/4 in | 2 | 12000-16000 | not given | 200-600 ipm; max cut 1/8 in | same | web table | page read |
| MDF | Onsrud 48-700 series | 1/2 in | not stated | not stated | .006-.008 in | | https://www.woodweb.com/knowledge_base_images/zp/Chipload_MDF.pdf | vendor (Onsrud via WoodWeb) | snippet |
| Laminated plywood | Onsrud 60-100C series | 1/2 in | not stated | not stated | .021-.023 in | 2 x D: -25 %; 3 x D: -50 % | https://www.woodweb.com/knowledge_base_images/zp/Chipload_Plywood.pdf | vendor (Onsrud) | snippet |
| Softwood | Onsrud 52-200 series | 1/4 in | not stated | not stated | .007-.009 in | | https://www.woodweb.com/knowledge_base_images/zp/Chipload_Softwood.pdf | vendor (Onsrud) | snippet |
| Softwood / hardwood / laminate / MDF | Amana solid carbide spiral plunge | 1/4 in | 2 | 18000 | .005-.007 in (plywood .006-.008) | 1 x D; 2 x D -25 %; 3 x D -50 % | https://eoasaw.com/images/Solid Carbide Spektra Coated Spiral Plunge Speed and Feed Chart.pdf | vendor (Amana) | snippet |
| Solid wood | Amana spiral plunge | 1/2 in / 1/8 in | 2 | 18000 (inf. from chart family) | .007-.009 / .003-.005 in | 250-320 / 110-180 ipm | https://cdn.shopify.com/s/files/1/1558/5651/files/Solid-Carbide-Spiral-Plunge-Solid-Wood-v5.pdf | vendor (Amana) | snippet |
| Hardwood / MDF | Amana spiral ball nose | 1/16, 1/8, 1/4 in | 2 | 18000 | hardwood .002-.004, .003-.005, .005-.007; MDF .003-.005, .005-.007, .006-.008 in | | https://es.global.toolstoday.com/content/ProductFile/Attachments/Spiral-Ball-Nose-Speed-Chart.pdf | vendor (Amana) | snippet |
| Wood / MDF | Amana Spektra spiral ball nose | 1/4 in | 2 | 18000 | .00075-.002 (older) vs .007-.009 in (v15) | versions conflict | https://www.amanatool.com/pub/media/productattachments/Spektra-Coated-3D-Profiling-Feed-Chip-Load-Chart-v15.pdf | vendor (Amana) | snippet |
| MDF | Amana, ToolsToday example | 3/8 in | 2 | not stated | not stated | 260 ipm feed; 130 ipm plunge | https://toolstoday.com/t-video-how-to-calculate-feeds-and-speeds | vendor | snippet |
| Any | Amana charts, ramp rule | | | | | ramp = feed / 2 (one chart); feed / flutes (another chart) | https://es.global.toolstoday.com/content/ProductFile/Attachments/Feed-Speed-for-Amana-Tool-Solid-Carbide-Spiral-Plunge-2-3-Flute.pdf | vendor (Amana) | snippet |
| Any | Freud CNC chart, depth rule | | | | | 2 x D: reduce chip load at least 25 %; 3 x D: at least 50 % | https://www.freudtools.com/public/assets/freud/downloadables/freudtools-router-bit-feed-and-speed-for-cnc-20170822.pdf | vendor (Freud) | snippet |
| Plywood | Vortex compression spiral | 1/2 in | not stated | Vortex range 10000-20000 | .021-.023 in | author derated to .0193 for 1 x D full slot | https://cnccookbook.com/cnc-router-feeds-and-speeds-from-manufacturers-recommendations | web article citing vendor | page read |
| Melamine / laminated particleboard | Compression | 3/8 in | 2 | 16000-18000 | not given | 500-600 ipm (400 ipm for 1/4 in bits) | https://woodweb.com/knowledge_base/Tooling_and_Settings_for_Machining_Laminated.html | practitioner forum | page read |

### W.3 Saw blades and boring bits

| Material | Tool kind | Size | Quantity | Value | Unit | Source | Type | Confidence |
|---|---|---|---|---|---|---|---|---|
| Softwood | Circular saw blade | n/a | Cutting speed | 60-100 | m/s | https://bioresources.cnr.ncsu.edu/BioRes_09/BioRes_09_1_1417_Krilek_KK_Energetic_Consumption_Woodcutting_Process_4913.pdf (Krilek et al. 2014) and https://www.agriculturejournals.cz/publicFiles/94_2009-JFS.pdf | paper | snippet |
| Hard and exotic wood | Circular saw blade | n/a | Cutting speed | 50-85 | m/s | same | paper | snippet |
| Spruce, beech | Circular saw blade, 54 T | 600 mm | Test settings | vc 60, 70, 80 m/s; vf 6 and 12 m/min | mixed | https://bioresources.cnr.ncsu.edu/?p=32790 (Kovac et al. 2021, BioResources 16(1) 1029-1041) | paper | page read |
| Spruce, beech | same | 600 mm | Feed per tooth | about 0.06 and 0.12 at vc 60 m/s | mm/tooth | derived from Kovac settings (inf.) | inference | n/a |
| Wood | Forstner bit | up to 1 in / larger | rpm | 1000-1500 / 500-1000 | rpm | https://answers.toolstoday.com/6506422/What-speed-should-you-run-a-Forstner-bit | vendor (Amana retailer) | snippet |

## Table R: robot notes

| Topic | Finding | Number | Source | Type | Confidence |
|---|---|---|---|---|---|
| Robot vs machine stiffness | Robots are far less stiff. Strategies: posture planning, dynamic milling (slow feed where deflection builds, steady radial engagement), stability lobes, simulation first. Quote: "optimize the robot's joint configuration to maximize the stiffness in the direction of cutting force" | robots about 0.1-1 N/um; machine tools above 50 N/um | https://robodk.com/blog/high-efficiency-machining-robots/ | vendor (RoboDK) | page read |
| Stiffness review | Static stiffness of serial industrial robots | 1e5-1e6 N/m; 500 N gives about 1 mm robot error vs below 0.01 mm on CNC | https://www.mdpi.com/2076-3417/12/17/8719 (Appl. Sci. 2022) | paper | snippet (403 on fetch) |
| Feed direction | Chatter limit depends on feed direction; low-frequency robot modes govern at low spindle speed | none in snippet | https://par.nsf.gov/servlets/purl/10543730 | paper | snippet |
| Critical depth | Robot milling critical depth of cut. Material not stated (metal assumed, inf.) | 0.35-0.93 mm at 2000 rpm; larger in Y feed than X | https://research.buaa.edu.cn/en/publications/dynamics-and-chatter-analysis-in-robotic-milling/ (Yang, Pei, Wang 2018) | paper | snippet |
| Marble, robot chatter | 2-DOF zeroth-order chatter model for robot milling of natural marble; force coefficients from robot tests; effect of parameters and pose studied | abstract gives no numbers | https://www.mdpi.com/2075-1702/12/12/942 (Huang, Huang, Huang, Yin 2024, Machines 12(12) 942); abstract read at https://library.kab.ac.ug/Record/doaj-art-d60610c9b91f498b9fbcacd7fa1a0554/Description | paper | abstract read |
| Marble relief, robot | Contour error vs spindle speed, feed, milling width, depth; optimum chosen and verified. Quote: "reasonable setting of process parameters is an essential link in the processing of white marble" | abstract gives no numbers | https://scientiairanica.sharif.edu/article_23422.html (Yin, Zhang 2024, Sci. Iran. 31(16) 1431-1449) | paper | abstract read |
| Force control | Position-controlled robot milling uses conservative feed; force control adjusts feed to hold a target force | none in snippet | https://arxiv.org/pdf/1409.6231 | paper | snippet |
| Low forces | High cutting speed with low feed per tooth and low depth keeps forces on the robot low | none in snippet | https://scielo.br/j/jbsmse/a/n7fhtjxn69TR4CvrnrXM6wx/?lang=en | paper | snippet |
| Wood, robot | KUKA KR 16-2 with spindle, 103 milling tests for timber joints | no feed data in snippet | https://www.daaam.info/Downloads/Pdfs/proceedings/proceedings_2024/034.pdf | paper | snippet |
| Soft materials | Wood, plastic, foam, aluminium can be robot-machined with accuracy close to machine tools | none | https://orbi.umons.ac.be/bitstream/20.500.12907/30640/1/RCIM_ModellingDynamicsRobot_HNH_2019.pdf | paper | snippet (attribution from search summary) |
| UR application note | None found for milling feeds or depth on UR arms | n/a | n/a | gap | n/a |
| KUKA application note | None found | n/a | n/a | gap | n/a |
| Numeric derating factor | No source found that gives a robot feed or depth derating factor | n/a | n/a | gap | n/a |

## How lithocam should use this (suggestion)

1. Core material table. Key each row by (material_class, tool_kind). Store each quantity as a range {min, max, unit} plus a list of source records {url, source_type, confidence, quote_or_note, retrieved_date}. A row with no source stays unset (0 today).
2. Material classes. Start with the classes the sources actually separate: hard granite, mid granite, soft granite, quartzite, engineered quartz, marble hard, marble soft, travertine (grouped with soft marble by one source), limestone, sandstone; wood: softwood, hardwood, soft plywood, laminated plywood, MDF / particleboard, HPL. Charnockite maps to "hard granite" only as a user choice, never as a default (no charnockite source found).
3. Surface speed to rpm. For diamond tools, store surface speed in m/s where a source gives it, and derive rpm = v x 60000 / (pi x D_mm). Where a source gives rpm at a stated diameter, store the rpm and the diameter, and show the derived m/s as "(derived)". Do not rescale an rpm to another diameter silently.
4. Chip-load feed. For flute tools (wood), feed = rpm x flutes x chip_load. Apply the depth rule from Amana, Onsrud and Freud: 1 x D full chip load, 2 x D minus 25 %, 3 x D minus 50 %. Plunge or ramp default = feed / 2 (Amana), shown with its source. Segmented diamond tools have no meaningful flute count; store their feed directly in mm/min.
5. Robot derating. No source supports a numeric factor. Ship the robot profile with derate = unset, and require the user to enter it. Show the RoboDK stiffness comparison (0.1-1 vs above 50 N/um) as context text only. Point to posture planning and stability lobes as the documented methods.
6. GUI. Next to each number show: value range, unit, source type badge (vendor, vendor blog, paper, web table), confidence (page read or snippet), and a link. Flag rows with known conflicts (see below) with a warning icon. Flag maximum ratings (cup wheels, pads) as "maximum, not working value". Let the user pin their own tested value, stored as source_type = "user test" with date.

## Gaps

- Forest Products Laboratory Wood Handbook machining chapter: not retrieved. No FPL numbers in this file.
- Tyrolit, Diamant Boart, Weha (maker pages), Diarex, Pellegrini setting tables: not found as readable pages. Weha values come from dealers.
- Charnockite: no source. Slate: no numeric source. Travertine: only grouped with soft marble.
- PCD tools for soft stone: no stone-specific data found (Amana PCD charts are for wood and composites).
- Calibrating heads: no peripheral speed or removal per head found.
- Waterjet: no granite data beyond 30 mm; no OMAX or Flow table read.
- Profile wheels and calibrating: no maker data sheet read.
- Robot: no UR or KUKA application note on milling parameters; no derating factor; the two marble robot papers give no numbers in their abstracts.
- Onsrud, Amana, Freud, KMT values come from PDFs that I did not read in full (search snippets only).

## Conflicts

- Granite blade peripheral speed: Sang 25-40 m/s by granite class vs Dynamic Stone Tools 45-60 m/s. Karakurt 2014 optimum is 25 m/s, and a 400 mm lab blade ran at 30 m/s. The 45-60 m/s figure is the outlier.
- Bridge saw feed in granite: Dynamic Stone Tools 1-2 in/min (25-51 mm/min) vs Sang 200-500 mm/min at 6 mm depth vs Karakurt 700 mm/min at 20 mm depth. The 1-2 in/min figure looks wrong (inf.).
- Finger bit rpm: Weha dealers 4500-8500 rpm for 20-22 mm bits vs Dynamic Stone Tools 8000-16000 rpm "CNC spindle" with no diameter.
- Weha Top Sprinter rpm: 5000-5800 vs 5000-8500 at two dealers.
- Amana Spektra 1/4 in ball nose: .00075-.002 in vs .007-.009 in across chart versions.
- Engineers Edge MDF table repeats the hardwood table exactly. One of them is likely a copy error (inf.).
- Wire tension: Konstanty 2000-2400 N vs vendor 100-300 kg (about 1000-3000 N). Vendor attribution unclear, so the vendor value is not tabled.
- Sang labels peripheral-speed-like values as "feeding speed (m/s)".
- MK Diamond 12 in core bit max printed as 30 rpm.

## Process notes

- Several pages returned 403 or 429 (diamondtoolstore, stonegatetooling, amastone, mdpi, doaj). Their values are snippet-only.
- Pages on `slabwise.com`, `dinosawweb-evi.pages.dev` and similar sites appeared in searches with unattributed numbers. I used only one slabwise table (polishing rpm by grit, flagged) and no dinosaw numbers.
