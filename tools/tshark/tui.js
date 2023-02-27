import blessed from 'blessed';
import * as contrib from 'blessed-contrib';

export class TuiDashboardBase {
    screen;
    grid;
    shutdown_cb; /**< Optional callback function to execute on user-requested exit */

    constructor() {
        // Setup main grid
        this.screen = blessed.screen();
        this.grid = new contrib.grid({
            rows: 20,
            cols: 10,
            screen: this.screen
        });

        // Register keyboard shortcuts
        this.screen.key(['escape', 'q', 'C-c'], (ch, key) => {
            this.exit();
            if (this.shutdown_cb) {
                this.shutdown_cb();
            } else {
                return process.exit(0);
            }
        });
        this.screen.key(['tab'], (ch,key) => {
            this.focused_idx = (this.focused_idx+1) % this.foci.length;
            this.foci[this.focused_idx].focus();
        });
    }

    /** Refresh dashboard from initially set pktsSets structure to display latest data
     *
     * This function can/should be called after pkt receipt, or at a regular interval.
     */
    refresh() {
        this.screen.render();
    }

    exit() {
        if (!this._exited) {
            this._exited = true;
            this.screen.destroy();
            if (this.intervalID)
                clearInterval(this.intervalD);
        }
    }

    bindTableKeys(table) {
        // TODO: Can this be merged into contrib.Table?  Only via forking ... blessed project seems to be unmaintained
        table.rows.key(['pageup'],   () => { table.rows.up(5); this.screen.render();} );
        table.rows.key(['pagedown'], () => { table.rows.down(5); this.screen.render();} );
        table.rows.key(['home'],     () => { table.rows.select(0); this.screen.render();} );
        table.rows.key(['end'],      () => { table.rows.select(table.rows.children.length-1); this.screen.render(); } );

        // Search Mode
        table.rows.key(['/'],        () => { // TODO: Search backwards option
            // Prompt for search term
            this.prompt.input("Enter search term", (e,term) => {
                if (term) {
                    // Find matching row
                    var idx = table.rows.children.findIndex( (r,idx) => idx > table.rows.selected && r.content.toLowerCase().includes(term.toLowerCase()) )
                    
                    // Select it
                    if (idx < 0) {
                        // No match
                        console.log("No match searching for ", term); // TODO: Display this
                    } else {
                        table.rows.select(idx);
                        this.screen.render();
                    }
                }
            });
        });
    }

}

export class TuiDashboard extends TuiDashboardBase {
    pkts = []; // Log of all pkts
    cnts; // Reference to object of counts to display
    
    constructor(cols, cnts={}) {
        super();
        this.cnts = cnts;
        this.cols = cols;
        
        // Create default widigits (future; make this configurable)
        this.log_table = this.grid.set(0,0,20,7, contrib.table, {
            keys: true,
            fg: 'green',
            label: 'Packets (since '+new Date().toUTCString()+')',
            columnSpacing: 1,
            columnWidth: this.cols.map(c => c.width)
        });
        this.cnt_table = this.grid.set(0,5,20,3, contrib.table, {
            keys: true,
            fg: 'green',
            label: "Stats",
            columnSpacing: 1,
            columnWidth: [10,10], // TODO
        });
        this.bindTableKeys(this.log_table);
        this.foci = [this.log_table, this.cnt_table];
    }
    refresh() {
        var log_hdrs = this.cols.map(c => c.title);
        var cfg_hdrs = ['Key','Value'];
        
        // is there a better way to update? to make this more generic/configurable?
        this.log_table.setData({ headers: log_hdrs, data: this.pkts });
        this.cnt_table.setData({ headers: cfg_hdrs, data: Object.keys(this.cnts).map(v => [v,this.cnts[v]]) });

        // TODO: Make this conditional on user not selecting otherwise
        this.log_table.rows.select(this.log_table.rows.children.length-1);
        
        super.refresh();
    }

    logPkt(cols) {
        this.pkts.push(cols);
    }
}
