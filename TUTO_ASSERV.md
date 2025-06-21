régler grossièrement PID theta seul
    forcer une petite rotation du robot à la main (genre 15deg)
    augmenter le P jusqu'a un avoir une oscillation amorti d'environ 1s
    augment le D jusqu'a ne plus avoir d'oscillation, retour amorti (quasi) sans dépassement

régler grossièrement le PID distance seul
    pousser le robot sur une petite distance (genre 2cm)
    même procédure que pour le PID theta

améliorer le PID theta
    laisser le PID theta grossier et PID distance grossier
    augmenter P pour dimuner le plus possible la zone morte autour de zéro (dépends des jeu mécanique)
    augmenter D pour garder le retrour armorti (quasi) sans dépassement

améliorer le PID distance
    laisser le PID theta amélioré
    même procédure que pour améliorer le PID theta

mesurer les jeux mécanique en theta et distance
    plot dist_error et theta_error
    permet de définir les paremètres THETA_FINISHED_WINDOW et DIST_FINISHED_WINDOW

vérifier que les PWM moteurs ne soient pas en saturation
    tester une rotation avec vitesse max et acceleration max
    tester une ligne avec vitesse max et acceleration max