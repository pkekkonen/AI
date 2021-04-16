#AI
Readme (DFS)
Ida Söderberg, Magnus Palmstierna, Paulina Lagebjer Kekkonen
Grupp 5
Github länk: https://github.com/pkekkonen/AI/tree/DFS/tanks_190324

I detta spel används inte timern, och tanken startar sin patrullering på en gång. När den ser en fiende åker den kortaste vägen hem baserat på utforskade noder och återgår sedan till att patrullera. Här används DFS för patrullering och A* för rapportering.

Brister i implementeringen: 
	Efter den åkt hem så stannar den inte i tre sekunder
	Den kan inte se framför sig, utan reagerar först när den krockar med en fiendetank.
	Om den stöter in i ett träd på ett visst vis, sätts denna nod som onåbar. Möjligen kan man dock nå samma nod från en annan riktning. 
	Om den stöter in i en tank sätts den noden som otillgänglig, trots att tanken eventuellt kan flytta sig senare.
	Den markerar heller inte platsen den sett den tidigare fiendetanken på som sämre. 
Uppnåelse av goal state kontrolleras inte.
Under rapportering kan tanken inte backa om den kört in i något.
