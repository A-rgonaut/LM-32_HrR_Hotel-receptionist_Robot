package com.example.l;

import java.io.File;
import java.util.Set;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;
import org.semanticweb.owlapi.apibinding.OWLManager;
import openllet.owlapi.OpenlletReasonerFactory;
import com.clarkparsia.owlapi.explanation.DefaultExplanationGenerator;
// import io.github.cdimascio.dotenv.Dotenv;

public class SpiegamiTutto {
    public static void main(String[] args) {

        if (args.length < 3) {
            System.err.println("Errore: parametri insufficienti. Uso: java -jar reasoner.jar <PathToOwlFile> <Individual> <Class>");
            System.exit(1);
        }
        String owlFilePath     = args[0];
        String namedIndividual = args[1];
        String owlClassTarget  = args[2];

        OWLOntologyManager man = OWLManager.createOWLOntologyManager();
        // Dotenv dotenv = Dotenv.load();
        // String pathOnto = dotenv.get("PATH_ONTO");

        System.out.println("--- AVVIO SISTEMA DI RAGIONAMENTO ---");

        // File fileOntologia = new File(pathOnto);
        File fileOntologia = new File(owlFilePath);

        if (fileOntologia.exists()) {
            System.out.println("Il file " + fileOntologia.getName() + " è stato trovato nel percorso: " + fileOntologia.getAbsolutePath());
            try {
                // Carica l'ontologia dal file
                OWLOntology ontologia = man.loadOntologyFromOntologyDocument(fileOntologia);
                OWLDataFactory df = man.getOWLDataFactory();
                System.out.println("Ontologia caricata con successo!");
                System.out.println("-> IRI/Nome dell'Ontologia: " + ontologia.getOntologyID().getOntologyIRI().orElse(IRI.create("Nessun nome esplicito")));
                System.out.println("-> Numero totale di Assiomi: " + ontologia.getAxiomCount());
                System.out.println("-> Numero di Classi: " + ontologia.getClassesInSignature().size());
                System.out.println("-> Numero di Data Properties: " + ontologia.getDataPropertiesInSignature().size());
                System.out.println("-> Numero di Object Properties: " + ontologia.getObjectPropertiesInSignature().size());
                System.out.println("-> Numero di Individui (Named Individuals): " + ontologia.getIndividualsInSignature().size());
                System.out.println("\n### Esempio Classi Trovate ###");
                ontologia.getClassesInSignature().stream().forEach(owlClass -> System.out.println(" - " + owlClass.getIRI().getFragment()));
                String baseIRI = ontologia.getOntologyID().getOntologyIRI()
                    .map(iri -> iri.toString())
                    .orElse("http://www.semanticweb.org/untitled-ontology-6"); // Fallback al tuo IRI esempio

                // Assicurati che l'IRI finisca con # o /
                if (!baseIRI.endsWith("#") && !baseIRI.endsWith("/")) {
                    baseIRI += "#";
                }

                // 4. Inizializzazione del Reasoner (Openllet/Pellet per SWRL)
                // Usiamo la Factory perché il generatore di spiegazioni deve poterne creare nuove istanze
                OWLReasonerFactory reasonerFactory = OpenlletReasonerFactory.getInstance();
                OWLReasoner reasoner = reasonerFactory.createReasoner(ontologia, new SimpleConfiguration());

                // Verifica coerenza iniziale
                if (!reasoner.isConsistent()) {
                    System.err.println("L'ontologia è inconsistente!");
                    return;
                }

                // 5. Definizione dell'Assioma da Spiegare
                // Vogliamo spiegare: PersonaEmergenza(luigi)

                // String namedIndividual= "Luigi";
                // String owlclass="PersonaEmergenza";
                // OWLNamedIndividual luigi = df.getOWLNamedIndividual(IRI.create(baseIRI + namedIndividual)); // O il nome esatto nell'ontologia
                // OWLClass personaEmergenza = df.getOWLClass(IRI.create(baseIRI + owlclass));

                // OWLClassAssertionAxiom assiomaDaSpiegare = df.getOWLClassAssertionAxiom(personaEmergenza, luigi);

                OWLNamedIndividual individuo = df.getOWLNamedIndividual(IRI.create(baseIRI + namedIndividual));
                OWLClass classe = df.getOWLClass(IRI.create(baseIRI + owlClassTarget));
                OWLClassAssertionAxiom assiomaDaSpiegare = df.getOWLClassAssertionAxiom(classe, individuo);

                System.out.println("\n--- Verifica Inferenza ---");
                // Controlliamo prima se il reasoner lo deduce davvero
                if (reasoner.isEntailed(assiomaDaSpiegare)) {

                    System.out.println("Generazione della spiegazione in corso (potrebbe richiedere tempo)...");

                    // 6. Generazione della Spiegazione
                    // DefaultExplanationGenerator usa una tecnica "black box" creando istanze del reasoner
                    DefaultExplanationGenerator gen = new DefaultExplanationGenerator(
                        man, reasonerFactory, ontologia, null);

                    // Otteniamo il set di assiomi che causano l'inferenza
                    Set<OWLAxiom> spiegazione = gen.getExplanation(assiomaDaSpiegare);
                    System.out.println("\n### SPIEGAZIONE (PERCHÉ LUIGI È UNA PERSONA EMERGENZA?) ###");
                    System.out.println(spiegazione.toString()
                        .replace("[", "").replace("]", "")
                        .replace(",", "")
                        .replaceAll("https?://[^>]*[#/]", "")
                        .replaceAll("\\b(rdfs|swrla):", "")
                        .replaceAll("[<>]", "")
                        .replaceAll("\\^\\^xsd:\\w+", "")
                        .replaceAll("\"(.*?)\"", "$1")
                        .replaceAll("Annotation\\(isRuleEnabled.*?\\)", "")
                        .replaceAll("Annotation\\(comment.*?\\)", "")
                        .replaceAll("\\s+", " ")
                        .replace("( ", "(")
                        .trim());
                } else {
                    System.out.println("Il reasoner NON deduce assiomi inerenti");
                }
            } catch (OWLOntologyCreationException e) {  // Gestisci l'errore nel caso in cui il caricamento fallisca
                System.err.println("ERRORE: Non è stato possibile caricare l'ontologia.");
                e.printStackTrace();
            }
        } else {  // Cosa succede se .exists() è false
            System.err.println("ERRORE: Il file non è stato trovato.");
            System.err.println("Controllare il percorso assoluto: " + fileOntologia.getAbsolutePath());
        }
    }
}
