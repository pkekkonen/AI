# AI

Ida Söderberg, Magnus Palmstierna, Paulina Lagebjer Kekkonen
Grupp 5

Github länk: https://github.com/pkekkonen/AI/tree/main/tanks_190324

I detta spel används inte timern, och tanken startar sin patrullering på en gång. När den ser en fiende åker den kortaste vägen hem baserat på utforskade noder, står still i boet i tre sekunder och rapporterar, och återgår sedan till att patrullera. Här används LRTA* för patrullering och A* för rapportering.

Brister i implementeringen: 

	- Ibland under patrullering när den stöter ihop med ett träd så fastnar den och kan ej gå vidare.
	
	- Under rapportering kan tanken inte backa om den kört in i något.
	
	- Uppnåelse av goal state kontrolleras inte.
