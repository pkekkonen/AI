# AI

Ida Söderberg, Paulina Kekkonen, Magnus Palmstierna, grupp 5

https://github.com/pkekkonen/AI/tree/boid

Problemet som åtagits är att tanksen gemensamt ska uppfylla målet att patrullera hela spelplanen. Strategin som applicerades för att lösa detta problem var att implementera boids beteende för alla tanks i respektive team för att åstadkomma ett flockbeteende inom teamet. Tanksen väljer ett gemensamt mål inom respektive team genom LRTA*. Om tanksen med sin synsensor ser en tank eller ett träd framför sig så ska den se till att inte krocka med dessa. Spelet startar på en gång och teamen får poäng när de utforskar en nod de inte varit vid tidigare och det blir game over antingen när ett team nått över 200 poäng eller när tiden på 3 minuter går ut. Oftast på grund av att tiden tar slut, då tanksen tyvärr lätt fastnar, se Brister i implementeringen. Om tanksen fastnar, kan man antingen starta om spelet eller låta tiden gå ut själv. Då tanksen inte har som mål att oskadliggöra andra tanks finns inte förutsättningarna gällande skjutandet implementerat i detta program. 

Referenser:
Shiffman, D. (2012). The nature of code.
http://rosettacode.org/wiki/Bitmap/Midpoint_circle_algorithm#Java


Brister i implementeringen:
- Tanksen kan röra sig i sidled, och baklänges.
- Ibland fastnar tanksen.
- Fastän tanken kan se i en kon, kan den fortfarande bara se det allra närmaste objektet.

