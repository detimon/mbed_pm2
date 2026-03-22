# Session-Ende Routine

Der Benutzer möchte die aktuelle Session sauber abschliessen.
Falls $ARGUMENTS nicht leer ist, nutze es als Kontext für was heute gemacht wurde.

Führe diese Schritte der Reihe nach aus:

## Schritt 1: CLAUDE.md aktualisieren (Pflicht — immer geladen)

CLAUDE.md ist die einzige zuverlässige Wahrheitsquelle zwischen Sessions.
Aktualisiere diese drei Abschnitte:

**"## Aktueller Stand"** — Überschreibe den gesamten Abschnitt mit:
- Was zuletzt getestet/implementiert wurde (konkret, mit Parametern/Werten)
- Was funktioniert, was nicht funktioniert
- Datum der letzten Session (heute)

**"## Nächste Schritte"** — Überschreibe den Abschnitt so dass Schritt 1 der **einzige, konkrete, sofort ausführbare** nächste Schritt ist. Kein "eventuell", kein "vielleicht". Beispiel: nicht "Servo testen" sondern "TEST_SERVO aktivieren und 180° Servo A (PC_8) auf 50% Position testen"

**"## Offene Fragen"** — Geklärte Fragen entfernen, neue ergänzen.

**"## Aktive Entscheidungen"** — Falls sich etwas geändert hat, ergänzen.

## Schritt 2: Auto Memory speichern (Backup)

Speichere als project-memory: was heute entschieden/gelöst/getestet wurde. Konkrete Details: Werte, Pins, Fehlermeldungen, Erkenntnisse. Keine Duplikate zu CLAUDE.md — nur was darüber hinausgeht.

## Schritt 3: Ausgabe

Zeige:
- **Aktueller Stand:** [ein Satz was heute erreicht wurde]
- **CLAUDE.md:** [welche Abschnitte geändert wurden]
- **Nächstes Mal, Schritt 1:** [exakt der Text aus "Nächste Schritte" Punkt 1]
